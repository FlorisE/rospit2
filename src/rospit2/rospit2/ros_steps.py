# Copyright (c) 2020 AIST.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""ROS specific steps."""

import asyncio
import copy
import importlib
import os
import signal
import threading
import time
from multiprocessing import Pipe, Process

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

from rcl_interfaces.msg import ParameterType

from rclpy.duration import Duration

from ros2launch.api import MultipleLaunchFilesError
from ros2launch.api import get_share_file_path_from_package

from ros2param.api import call_get_parameters, \
                          call_set_parameters

from ros2run.api import get_executable_path

from ros2topic.api import qos_profile_from_short_keys

from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.utilities import get_message

from .declarative import Step
from .ros import get_field_or_message
from .ros_parameters import get_parameter, \
                            parse_and_store_csv_parameters, \
                            parse_and_store_xml_parameters, \
                            set_parameter


class GetParameter(Step):
    """Get a parameter from the parameter server."""

    def __init__(
            self, node, node_name, parameter_name, stored_parameters,
            get_parameters_override=None, store_as=None):
        """Initialize."""
        super().__init__()
        if node is None:
            raise ValueError(node)
        self.node = node  # node used for execution

        if node_name is None:
            raise ValueError(node_name)
        self.node_name = node_name  # node to get parameter value from

        if parameter_name is None:
            raise ValueError(parameter_name)
        self.parameter_name = parameter_name

        if stored_parameters is None:
            raise ValueError(stored_parameters)
        self.stored_parameters = stored_parameters  # dict to store value in

        if get_parameters_override is not None:
            self.call_get_parameters = get_parameters_override
        else:
            self.call_get_parameters = call_get_parameters

        self.store_as = store_as

    def execute(self):
        """Get the parameter value."""
        get_parameter(
            self.node,
            self.node_name,
            self.parameter_name,
            self.store_as,
            self.stored_parameters)


class SetParameter(Step):
    """Set a parameter on the parameter server."""

    def __init__(
            self, node, node_name, parameter_name, parameter_value,
            stored_parameters, set_parameters_override=None, store_as=None):
        """Initialize."""
        super().__init__()

        if node is None:
            raise ValueError(node)
        self.node = node  # node used for execution

        if node_name is None:
            raise ValueError(node_name)
        self.node_name = node_name  # node to get parameter value from

        if parameter_name is None:
            raise ValueError(parameter_name)
        self.parameter_name = parameter_name

        self.parameter_value = parameter_value

        if stored_parameters is None:
            raise ValueError(stored_parameters)
        self.stored_parameters = stored_parameters  # dict to store value in

        if set_parameters_override is not None:
            self.call_set_parameters = set_parameters_override
        else:
            self.call_set_parameters = call_set_parameters

        self.store_as = store_as

    def execute(self):
        """Set the parameter value."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(
            set_parameter(
                self.node,
                self.node_name,
                self.parameter_name,
                self.parameter_value,
                self.store_as,
                self.stored_parameters))


class LoadParametersFromFile(Step):
    """Loads multiple parameters at a time."""

    def __init__(self, node, path, stored_parameters):
        """Initialize."""
        super().__init__()

        if node is None:
            raise ValueError(node)
        self.node = node  # node used for execution

        if path is None:
            raise ValueError(path)
        self.path = path

        if stored_parameters is None:
            raise ValueError(stored_parameters)
        self.stored_parameters = stored_parameters


class LoadParametersFromXMLFile(LoadParametersFromFile):
    """Loads multiple parameters at a time, from a CSV file."""

    def __init__(self, node, path, stored_parameters):
        """Initialize."""
        super().__init__(node, path, stored_parameters)

    def execute(self):
        """Load the parameters."""
        parse_and_store_xml_parameters(
            self.node, self.path, self.stored_parameters)


class LoadParametersFromCSVFile(LoadParametersFromFile):
    """Loads multiple parameters at a time, from a CSV file."""

    def __init__(self, node, path, separator, stored_parameters):
        """Initialize."""
        super().__init__(node, path, stored_parameters)
        if separator is None:
            raise ValueError(separator)
        self.separator = separator

    def execute(self):
        """Load the parameters."""
        parse_and_store_csv_parameters(
            self.node,
            self.path,
            self.separator,
            self.stored_parameters)


class LaunchArgs:
    """Arguments for launching a ROS system."""

    def __init__(
            self, package_name, launch_file_name, launch_arguments, debug,
            print_, show_args, show_all_subprocesses_output):
        """Initialize."""
        self.package_name = package_name
        self.launch_file_name = launch_file_name
        self.launch_arguments = launch_arguments
        self.debug = debug
        self.print = print_
        self.show_args = show_args
        self.show_all_subprocesses_output = show_all_subprocesses_output


def launch(
        shutdown_pipe,
        package_name, launch_file_name, launch_arguments, debug):
    """Launch a ROS system."""
    event_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(event_loop)
    # based on ros2launch/command/launch.py:main
    if package_name is None:
        single_file = True
    else:
        single_file = False
        get_package_prefix(package_name)

    path = None
    launch_arguments = []
    if single_file:
        if os.path.exists(package_name):
            path = package_name
        else:
            raise ValueError(package_name)
        if launch_file_name is not None:
            launch_arguments.append(launch_file_name)
    else:
        try:
            path = get_share_file_path_from_package(
                package_name=package_name,
                file_name=launch_file_name)
        except PackageNotFoundError as exc:
            raise RuntimeError(
                "Package '{}' not found: {}".format(
                    package_name, exc))
        except (FileNotFoundError, MultipleLaunchFilesError) as exc:
            raise RuntimeError(str(exc))
    launch_arguments.extend(launch_arguments)
    launch_service = LaunchService(argv=launch_arguments, debug=debug)
    launch_description = LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                path
            ),
            launch_arguments={},
        ),
    ])
    launch_service.include_launch_description(launch_description)
    finished = False

    def shutdown():
        while not finished and not shutdown_pipe.poll(1):
            pass
        launch_service.shutdown()

    t = threading.Thread(target=shutdown)
    t.start()
    launch_service.run(shutdown_when_idle=True)
    finished = True
    t.join()
    event_loop.close()


class Launch(Step):
    """Run a ROS launch file."""

    def __init__(
            self,
            node,
            package_name=None,
            launch_file_name=None,
            launch_arguments=[],
            debug=False):
        """Initialize."""
        super().__init__()

        self.node = node
        self.package_name = package_name
        self.launch_file_name = launch_file_name
        self.launch_arguments = launch_arguments
        self.debug = debug
        self.process = None

    def execute(self):
        """Launch it."""
        if len(self.launch_arguments):
            self.node.get_logger().info(
                'Launching {} of package {} with arguments {}'.format(
                    self.launch_file_name,
                    self.package_name,
                    self.launch_arguments))
        else:
            self.node.get_logger().info(
                'Launching {} of package {}'.format(
                    self.launch_file_name, self.package_name))
        self.parent_shutdown, child_shutdown = Pipe()
        self.process = Process(
            target=launch, args=(
                child_shutdown, self.package_name, self.launch_file_name,
                self.launch_arguments, self.debug))
        self.process.start()

    def clean_up(self):
        """Clean up the step."""
        self.parent_shutdown.send(True)
        self.process.join()


def substitude_values(value_dict, param_dict):
    """Substitude variables in the parameters."""
    def get_result(param_buff):
        param_dict_key = param_buff.decode()
        value = param_dict[param_dict_key]
        if value.type == ParameterType.PARAMETER_BOOL:
            value = str(value.bool_value)
        elif value.type == ParameterType.PARAMETER_INTEGER:
            value = str(value.integer_value)
        elif value.type == ParameterType.PARAMETER_DOUBLE:
            value = str(value.double_value)
        elif value.type == ParameterType.PARAMETER_STRING:
            value = value.string_value
        else:
            raise RuntimeError(f'value is of unrecognized type {value.type}')
        return value.encode('utf-8')

    for key, value in value_dict.items():
        v_type = type(value)
        if v_type is dict:
            value_dict[key] = substitude_values(value, param_dict)
        elif v_type is str:
            i = 0
            dollar_found = False
            result = bytearray('', encoding='utf-8')
            param_buff = bytearray('', encoding='utf-8')
            while i < len(value):
                if not dollar_found:
                    if value[i] == '$':
                        dollar_found = True
                    else:
                        result += value[i].encode('utf-8')
                else:
                    is_space = value[i] == ' '
                    is_dollar = value[i] == '$'
                    if is_space or is_dollar:
                        result += get_result(param_buff)
                        param_buff = bytearray('', encoding='utf-8')
                        if is_space:
                            dollar_found = False
                            result += ' '.encode('utf-8')
                    else:
                        param_buff += value[i].encode('utf-8')
                i += 1
            if dollar_found:
                result += get_result(param_buff)
            value_dict[key] = result.decode()
        else:
            # no need to do anything
            pass
    return value_dict


class Publish(Step):
    """Publish a message to a topic."""

    def __init__(
            self, node, topic, msg_type, duration, rate, parameters,
            parameter_values, use_substitution,
            qos_profile_str='system_default',
            qos_reliability_str='system_default',
            qos_durability_str='system_default',
            save_result=False):
        """Publish a message to a topic."""
        super().__init__(save_result)

        self.node = node
        self.topic = topic
        self.msg_type = msg_type
        self.duration = 1 if duration is None else duration
        self.rate = 1 if rate is None else rate
        self.parameters = parameters
        self.parameter_values = parameter_values
        self.use_substitution = use_substitution
        self.qos_profile_str = qos_profile_str
        self.qos_reliability_str = qos_reliability_str
        self.qos_durability_str = qos_durability_str

    def execute(self):
        """Publish a message to a topic."""
        msg = ('Publishing message to topic {} '
               'for a duration of {} with a rate of {}')
        self.node.get_logger().info(
            msg.format(self.topic, self.duration, self.rate))
        once = self.duration == 1 and self.rate == 1
        qos_profile = \
            qos_profile_from_short_keys(self.qos_profile_str,
                                        reliability=self.qos_reliability_str,
                                        durability=self.qos_durability_str)
        msg_module = get_message(self.msg_type)
        pub = self.node.create_publisher(msg_module, self.topic, qos_profile)

        msg = msg_module()
        if self.use_substitution:
            # deep copy so we don't lose the variables for future executions
            parameters_copy = copy.deepcopy(self.parameters)
            substitude_values(parameters_copy, self.parameter_values)
            set_message_fields(msg, parameters_copy)
        else:
            set_message_fields(msg, self.parameters)

        if not once:
            clock = self.node.get_clock()
            sched_time = clock.now()
            end_time = sched_time + Duration(nanoseconds=self.duration * 10**9)

            while clock.now() < end_time:
                while clock.now() < sched_time:
                    time.sleep((sched_time - clock.now()).nanoseconds / 10**9)
                if clock.now() > end_time:
                    break
                pub.publish(msg)
                sched_time = sched_time + Duration(
                    nanoseconds=(1./self.rate) * 10**9)
        else:
            pub.publish(msg)

        self.node.destroy_publisher(pub)

        return True


class Run(Step):
    """Run a ROS package specific executable."""

    def __init__(self, node, package_name, executable_name, argv, blocking):
        """Initialize."""
        super().__init__()

        self.node = node
        self.package_name = package_name
        self.executable_name = executable_name
        self.argv = argv
        self.blocking = blocking
        self.process = None

    def execute(self):
        """Call the executable."""
        msg = 'Running executable {} of package {}'
        self.node.get_logger().info(
            msg.format(self.executable_name, self.package_name))
        path = get_executable_path(
                package_name=self.package_name,
                executable_name=self.executable_name)
        if path is None:
            raise FileNotFoundError(
                f'{self.package_name}/{self.executable_name}')
        self.process = asyncio.run(self._execute(path))

    async def _execute(self, path):
        """Execute asynchronously."""
        self.process = await asyncio.create_subprocess_exec(path)
        if self.blocking:
            await self.process.wait()
        return self.process

    def clean_up(self):
        """Terminate the executable, if it's still running."""
        if not self.process.returncode:
            self.process.send_signal(signal.SIGINT)


class TopicValue(object):
    """Message value."""

    def __init__(self, topic, field, test_suite):
        """Initialize."""
        self.topic = topic
        self.field = field
        self.test_suite = test_suite

    def get_value(self):
        """Get the value of the message."""
        message = self.test_suite.messages[self.topic]
        return get_field_or_message(message, self.field)


class StaticValue(object):
    """Static value."""

    def __init__(self, value, type_, test_suite):
        """Initialize."""
        self.value = value
        self.type = type_
        self.test_suite = test_suite

    def get_value(self):
        """Get the static value."""
        return self.value


def _fill_parameters(parameters):
    def _process(item):
        if isinstance(item, dict):
            return_value = {}
            for key, value in item.items():
                return_value[key] = _process(value)
            return return_value
        elif isinstance(item, TopicValue) or isinstance(item, StaticValue):
            return item.get_value()
        else:
            return item
    val = _process(parameters)
    return val


def _get_module(service_type):
    """Get the module for service_type."""
    try:
        parts = service_type.split('/')
        if len(parts) == 2:
            parts = [parts[0], 'srv', parts[1]]
        package_name = parts[0]
        module = importlib.import_module('.'.join(parts[:-1]))
        srv_name = parts[-1]
        srv_module = getattr(module, srv_name)
        if not package_name or not srv_module:
            raise ValueError()
    except ValueError:
        raise RuntimeError('The passed service type is invalid')
    return srv_module


def call_service(node, service_name, service_type, service_args):
    """Call a service. Mostly extracted from rosservice."""
    srv_module = _get_module(service_type)

    client = node.create_client(srv_module, service_name)

    request = srv_module.Request()

    try:
        set_message_fields(request, service_args)
    except Exception as e:
        raise e
        return 'Failed to populate field: {0}'.format(e)

    if not client.service_is_ready():
        client.wait_for_service()
    client.call(request)


class ServiceCall(Step):
    """Call a ROS Service."""

    def __init__(
            self,
            node,
            service,
            service_type,
            parameters=None,
            stored_parameters=None,
            use_substitution=False,
            save_result=False):
        """Call a ROS Service."""
        super().__init__(save_result)
        if parameters is None:
            parameters = {}
        self.node = node
        self.service = service
        self.service_type = service_type
        self.parameters = parameters
        self.stored_parameters = stored_parameters
        self.use_substitution = use_substitution

    def execute(self):
        """Call the service."""
        self.node.get_logger().info(
            'Calling service {}'.format(self.service))
        if self.use_substitution:
            # deep copy so we don't lose the variables for future executions
            parameters_copy = copy.deepcopy(self.parameters)
            substitude_values(parameters_copy, self.parameter_values)
            return call_service(
                self.node, self.service, self.service_type,
                _fill_parameters(parameters_copy))
        else:
            return call_service(
                self.node, self.service, self.service_type,
                _fill_parameters(self.parameters))


class Sleep(Step):
    """Sleep for a specified amount of time."""

    def __init__(self, node, time, unit='second'):
        """Initialize."""
        super().__init__()
        self.node = node
        if unit == 'second' or unit == 'seconds':
            self.time = time
        elif unit == 'minute' or unit == 'minutes':
            self.time = time * 60
        elif unit == 'hour' or unit == 'hours':
            self.time = time * 60 * 60
        else:
            raise ValueError('Supported units are second, minute and hour')

    def execute(self):
        """Sleep for the specified time."""
        self.node.get_logger().info(
            'Sleeping for {} second(s)'.format(self.time))

        rate = self.node.create_rate(1 / self.time)
        rate.sleep()

        self.node.get_logger().info('Finished sleeping')
