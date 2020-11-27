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

from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters

from rclpy.duration import Duration

from ros2launch.api import MultipleLaunchFilesError
from ros2launch.api import get_share_file_path_from_package

from ros2param.api import call_get_parameters, \
                          call_set_parameters, \
                          get_parameter_value

from ros2run.api import get_executable_path

from ros2topic.api import qos_profile_from_short_keys

from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.utilities import get_message

from .declarative import Step
from .ros import get_field_or_message


class GetParameter(Step):
    """Get a parameter from the parameter server."""

    def __init__(
            self, node, node_name, parameter_name, stored_parameters,
            get_parameters_override=None, store_as=None):
        """Initialize."""
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
        if self.store_as:
            self.node.get_logger().info(
                'Getting parameter {} of node {}, storing as {}'.format(
                    self.parameter_name, self.node_name, self.store_as))
        else:
            self.node.get_logger().info(
                'Getting parameter {} of node {}'.format(
                    self.parameter_name, self.node_name))

        client = self.node.create_client(
            GetParameters, f'{self.node_name}/get_parameters')
        ready = client.wait_for_service(timeout_sec=5.0)
        if not ready:
            raise RuntimeError('Wait for service timed out')

        request = GetParameters.Request()
        request.names = [self.parameter_name]
        result = client.call(request)
        key = self.store_as or f'/{self.node_name}/{self.parameter_name}'
        pvalue = result.values[0]
        self.stored_parameters[key] = pvalue
        if pvalue.type == ParameterType.PARAMETER_BOOL:
            label = 'Boolean value is:'
            value = pvalue.bool_value
        elif pvalue.type == ParameterType.PARAMETER_INTEGER:
            label = 'Integer value is:'
            value = pvalue.integer_value
        elif pvalue.type == ParameterType.PARAMETER_DOUBLE:
            label = 'Double value is:'
            value = pvalue.double_value
        elif pvalue.type == ParameterType.PARAMETER_STRING:
            label = 'String value is:'
            value = pvalue.string_value
        elif pvalue.type == ParameterType.PARAMETER_BYTE_ARRAY:
            label = 'Byte values are:'
            value = pvalue.byte_array_value
        elif pvalue.type == ParameterType.PARAMETER_BOOL_ARRAY:
            label = 'Boolean values are:'
            value = pvalue.bool_array_value
        elif pvalue.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            label = 'Integer values are:'
            value = pvalue.integer_array_value
        elif pvalue.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            label = 'Double values are:'
            value = pvalue.double_array_value
        elif pvalue.type == ParameterType.PARAMETER_STRING_ARRAY:
            label = 'String values are:'
            value = pvalue.string_array_value
        elif pvalue.type == ParameterType.PARAMETER_NOT_SET:
            label = 'Parameter not set.'
            value = None
        else:
            raise ValueError(
                f"Unknown parameter type '{pvalue.type}' for key {key}")
        self.node.get_logger().info(f'{label} {value}')


class SetParameter(Step):
    """Set a parameter on the parameter server."""

    def __init__(
            self, node, node_name, parameter_name, parameter_value,
            stored_parameters, set_parameters_override=None, store_as=None):
        """Initialize."""
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
        if self.store_as:
            self.node.get_logger().info(
                'Setting parameter {} of node {} to {}, storing as {}'.format(
                    self.parameter_name, self.node_name,
                    self.parameter_value, self.store_as))
        else:
            self.node.get_logger().info(
                'Setting parameter {} of node {} to {}'.format(
                    self.parameter_name, self.node_name, self.parameter_value))

        client = self.node.create_client(
            SetParameters, f'{self.node_name}/set_parameters')
        ready = client.wait_for_service(timeout_sec=5.0)
        if not ready:
            raise RuntimeError('Wait for service timed out')

        parameter_encoded = get_parameter_value(
            string_value=self.parameter_value)

        parameter = Parameter()
        parameter.name = self.parameter_name
        parameter.value = parameter_encoded

        request = SetParameters.Request()
        request.parameters = [parameter]
        client.call(request)

        key = self.store_as or f'/{self.node_name}/{self.parameter_name}'
        self.stored_parameters[key] = parameter_encoded


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


class Publish(Step):
    """Publish a message to a topic."""

    def __init__(self, node, topic, msg_type, duration, rate, parameters,
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
        super().__init__(False)
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

    def __init__(self, node, service, service_type,
                 parameters=None, save_result=False):
        """Call a ROS Service."""
        super().__init__(save_result)
        if parameters is None:
            parameters = {}
        self.node = node
        self.service = service
        self.service_type = service_type
        self.parameters = parameters

    def execute(self):
        """Call the service."""
        self.node.get_logger().info(
            'Calling service {}'.format(self.service))
        return call_service(
            self.node, self.service, self.service_type,
            _fill_parameters(self.parameters))


class Sleep(Step):
    """Sleep for a specified amount of time."""

    def __init__(self, node, time, unit='second'):
        """Initialize."""
        super().__init__(False)
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
