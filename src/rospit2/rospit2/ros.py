# Copyright (c) 2020 AIST.
# National Institute of Advanced Industrial Science and Technology.
#
# License: MIT.

"""ROS specific implementation of the testing framework."""


import importlib
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.duration import Duration
from rclpy.node import Node

from ros2topic.api import qos_profile_from_short_keys

from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.utilities import get_message

from rospit_msgs.action import ExecuteXMLTestSuite
from rospit_msgs.msg import ConditionEvaluationPairStamped

from .binary import BinaryMeasurement
from .declarative import DeclarativeTestCase, Step
from .framework import Evaluation, Evaluator, Measurement, \
                       TestSuite, get_logger
from .numeric import BothLimitsCondition, BothLimitsEvaluator, \
                     EqualToCondition, EqualToEvaluator, \
                     GreaterThanCondition, GreaterThanEvaluator, \
                     GreaterThanOrEqualToCondition, \
                     GreaterThanOrEqualToEvaluator, \
                     LessThanCondition, LessThanEvaluator, \
                     LessThanOrEqualToCondition, \
                     LessThanOrEqualToEvaluator, \
                     LowerLimitCondition, LowerLimitEvaluator, \
                     NotEqualToCondition, NotEqualToEvaluator, \
                     NumericMeasurement, \
                     UpperLimitCondition, UpperLimitEvaluator
from .test_runner import map_test_suite_report


INVARIANT_EVALUATIONS_TOPIC = '/invariant_evaluations'


class SubscriptionManager(object):
    """Manages subscriptions."""

    def __init__(self, node, subscribers):
        """Initialize."""
        self.node = node
        self.subscribers = subscribers
        self.initialized_subscribers = {}

    def get_subscriber(self, topic, msg_type, func):
        """Get a ROS subscriber for the topic and type."""
        return self.node.create_subscription(msg_type, topic, func, 10)

    def delete_subscriber(self, subscription):
        """Delete a subscriber."""
        self.node.destroy_subscription(subscription)

    def delete_subscribers(self):
        """Delete the subscribers."""
        for topic, subscription in self.initialized_subscribers.items():
            self.delete_subscriber(subscription)

    def initialize_subscribers(self):
        """Initialize the subscribers."""
        for topic, msg_type in self.subscribers.msg_value_subscribers:
            if topic not in self.initialized_subscribers:
                self.initialized_subscribers[topic] = self.get_subscriber(
                    topic, msg_type, self.store_message, topic)
            if topic not in self.node.msg_value_subscribers:
                self.node.msg_value_subscribers[topic] = \
                        self.initialized_subscribers[topic]
        for topic, msg_type in self.subscribers.msg_received_subscribers:
            if topic not in self.initialized_subscribers:
                self.initialized_subscribers[topic] = self.get_subscriber(
                    topic, msg_type, self.store_message, topic)
            if topic not in self.node.msg_received_subscribers:
                self.node.msg_received_subscribers[topic] = \
                        self.initialized_subscribers[topic]


class ROSTestSuite(TestSuite):
    """A ROS specific test suite."""

    def __init__(self, node, subscribers, name=''):
        """Initialize."""
        TestSuite.__init__(self, name)
        self.node = node
        self.messages = {}
        self.message_received_on = set()
        self.msg_value_subscribers = {}
        self.msg_received_subscribers = {}
        self.subscription_manager = SubscriptionManager(self.node, subscribers)

    def run(self, logger):
        """
        Run the test suite.

        Wraps the super method to add subscription management.
        """
        self.subscription_manager.initialize_subscribers()
        report = super().run(logger)
        self.subscription_manager.delete_subscribers()
        return report

    def store_message(self, data, topic):
        """Store the actual message."""
        self.messages[topic] = data

    def store_msg_received_on(self, data, topic):
        """Store on which topics messages have been received."""
        self.message_received_on.add(topic)

    def report_executing(self, test_case):
        """Report which test suite is running to the action server."""
        self.node.report_executing(self.name, test_case)


class ROSDeclarativeTestCase(DeclarativeTestCase):
    """A declarative test case for ROS."""

    def __init__(self, test_suite, run_steps=None, set_up_steps=None,
                 tear_down_steps=None, name='', preconditions=None,
                 invariants=None, postconditions=None,
                 wait_for_preconditions=False,
                 sleep_rate=0.1, depends_on=None):
        """Initialize."""
        super().__init__(run_steps, set_up_steps, tear_down_steps, name,
                         preconditions, invariants, postconditions,
                         wait_for_preconditions, sleep_rate, depends_on)
        self.test_suite = test_suite
        self.subscribers = []
        self.subscription_manager = test_suite.subscription_manager

    def run(self):
        """Run the ROS declarative test case. Reports to the action server."""
        self.test_suite.report_executing(self.name)
        super().run()

    def start_invariant_monitoring(self):
        """Start monitoring the invariants."""
        for invariant in self.invariants:
            self.subscribers.append(self.get_invariant_subscriber(invariant))

    def stop_invariant_monitoring(self):
        """Stop monitoring the invariants."""
        for subscriber in self.subscribers:
            self.subscription_manager.delete_subscriber(subscriber)

    def get_invariant_subscriber(self, invariant):
        """Create a subscriber for monitoring the invariant."""
        def subscribe(data):
            """Process data received on the invariant."""
            evaluation = invariant.evaluator.evaluate(
                invariant.condition,
                get_field_or_message(data, invariant.evaluator.field))
            self.invariants_evaluations[invariant].append(evaluation)
            if not evaluation.nominal:
                self.invariant_failed = True

        msg_module = get_message(invariant.evaluator.topic_type)
        return self.subscription_manager.get_subscriber(
            invariant.evaluator.topic, msg_module, subscribe)


class ROSInvariant(object):
    """Some condition that should hold throughout execution of a test case."""

    def __init__(self, condition, evaluator, topic, msg_type):
        """Initialize."""
        self.condition = condition
        self.evaluator = evaluator
        self.topic = topic
        self.msg_type = msg_type

    def call_evaluator_with_data(self, data):
        """Call the evaluator with supplied data."""
        return self.evaluator.call_evaluator_with_data(data)

    def evaluate_measurement(self, measurement):
        """Evaluate the measurement."""
        return self.evaluator.evaluate(self.condition, measurement)

    def evaluate(self, data):
        """Evaluate the invariant."""
        measurement = self.call_evaluator_with_data(data)
        return self.evaluate_measurement(measurement)


class ROSTestRunnerNode(Node):
    """A node that runs tests and publishes their results."""

    def __init__(self, executor):
        """Initialize."""
        super().__init__('test_runner')
        self.node_executor = executor
        self.invariant_evaluations = []

        self.invariant_evaluation_subscription = self.create_subscription(
            ConditionEvaluationPairStamped, INVARIANT_EVALUATIONS_TOPIC,
            self.add_invariant_evaluation, 10)
        # prevent unused variable warning
        self.invariant_evaluation_subscription
        self.spinning = False
        self.last_test_suite = None
        self._action_server = ActionServer(
                self,
                ExecuteXMLTestSuite,
                'execute_xml_test_suite',
                self.execute_xml_test_suite)
        self.active_goal_handle = None

    def execute_xml_test_suite(self, goal_handle):
        """
        Execute a test suite specified in an XML file.

        Request should be a string specifying the path to the test to run.
        """
        self.active_goal_handle = goal_handle
        self.get_logger().info('Executing test suite')
        result = ExecuteXMLTestSuite.Result()

        from rospit2.rospit_xml import get_test_suite_from_xml_path
        if not goal_handle.request.path:
            result.success = False
            result.error = 'No path to test description specified'
            return result

        parser = get_test_suite_from_xml_path(
            self, goal_handle.request.path, True)
        if not parser:
            result.success = False
            result.error = 'Failed to parse the file specified at path'
            return result

        self.last_test_suite = parser.parse()

        if self.last_test_suite is None:
            result.success = False
            result.error = 'No test suite loaded, call execute_xml_test_suite'
            return result

        report = self.last_test_suite.run(self.get_logger())
        mapped_report = map_test_suite_report(report)
        goal_handle.succeed()
        result.success = True
        result.report = mapped_report
        return result

    def add_invariant_evaluation(self, evaluation):
        """Store the invariant evaluation."""
        self.invariant_evaluations.append(evaluation)

    def spin(self):
        """Spin the node."""
        self.spinning = True
        self.get_logger().info('Test runner ready')
        rclpy.spin(self, self.node_executor)

    def report_executing(self, test_suite, test_case):
        """Report to action server which suite and case are being executed."""
        if not self.active_goal_handle:
            self.get_logger().error('not currently executing a goal')
        feedback_msg = ExecuteXMLTestSuite.Feedback()
        feedback_msg.state = 'Executing'
        feedback_msg.active_test_suite_name = test_suite
        feedback_msg.active_test_case_name = test_case
        self.active_goal_handle.publish_feedback(feedback_msg)


class MessageValue(object):
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


def get_field_or_message(message, field_str):
    """Get the field value or entire message if it is already a leaf."""
    data = message
    if field_str is not None:
        fields = field_str.split('/')
        while len(fields) > 0:
            field = fields.pop(0)
            data = getattr(data, field)
    return data


class MessageEvaluatorBase(Evaluator):
    """Base class for message evaluation."""

    def __init__(self, node, topic, topic_type, field=None):
        """Initialize the message evaluator."""
        self.received = False
        self.node = node
        self.topic = topic
        self.topic_type = topic_type
        self.field = field
        self.data = None
        msg_type = get_message(topic_type)
        self.subscriber = self.node.create_subscription(
            msg_type, topic, self.callback, 10)

    def callback(self, data):
        """Fill data and mark as received."""
        self.data = get_field_or_message(data, self.field)
        self.received = True


class MessageReceivedEvaluator(MessageEvaluatorBase):
    """Evaluate whether a message has been received on the topic."""

    def __init__(self, node, topic, topic_type, field=None):
        """Initialize."""
        MessageEvaluatorBase.__init__(self, node, topic, topic_type, field)

    def evaluate_internal(self, condition, measurement=None):
        """Internally evaluate the message."""
        if measurement is None:
            measurement = BinaryMeasurement(self.received)
        return Evaluation(
            measurement, condition, self.received == condition.value)


class MessageEvaluator(MessageEvaluatorBase):
    """Evaluate the content of the message."""

    def __init__(self, node, topic, topic_type, field=None):
        """Initialize."""
        MessageEvaluatorBase.__init__(self, node, topic, topic_type, field)

    def evaluate_internal(self, condition, measurement=None):
        """Internally evaluate the message."""
        if measurement is None:
            while self.data is None:
                time.sleep(1)
            measurement = self.data
        return Evaluation(measurement, condition, self.data == condition.value)


class ExecutionReturnedEvaluator(Evaluator):
    """Evaluator for whether execution has returned."""

    def __init__(self, test_case, field=None):
        """Initialize."""
        self.test_case = test_case
        self.field = field

    def evaluate_internal(self, condition, measurement=None):
        """Internally evaluate the execution."""
        if measurement is None:
            current = self.test_case.execution_result
            if self.field is not None:
                fields = self.field.split('/')
                for field in fields:
                    current = getattr(current, field)
            measurement = Measurement(current)

        evaluation = Evaluation(
            measurement, condition, measurement.value == condition.value)
        return evaluation


class NumericMessageEvaluator(MessageEvaluatorBase):
    """Evaluator for numeric messages."""

    def __init__(self, node, topic, topic_type, field=None):
        """Initialize."""
        MessageEvaluatorBase.__init__(self, node, topic, topic_type, field)

    def evaluate_internal(self, condition, measurement=None):
        """Internally evaluate the numeric message."""
        if measurement is None:
            while self.data is None:
                time.sleep(1)
            measurement = NumericMeasurement(self.data)
        type_map = {
            BothLimitsCondition: BothLimitsEvaluator,
            UpperLimitCondition: UpperLimitEvaluator,
            LowerLimitCondition: LowerLimitEvaluator,
            GreaterThanCondition: GreaterThanEvaluator,
            GreaterThanOrEqualToCondition: GreaterThanOrEqualToEvaluator,
            EqualToCondition: EqualToEvaluator,
            NotEqualToCondition: NotEqualToEvaluator,
            LessThanOrEqualToCondition: LessThanOrEqualToEvaluator,
            LessThanCondition: LessThanEvaluator
        }

        evaluator_type = None
        for key, value in type_map.items():
            if isinstance(condition, key):
                evaluator_type = value
                break
        if evaluator_type is None:
            raise ValueError('Condition is of unknown type')

        evaluation = evaluator_type(lambda: self.data).evaluate(
            condition, measurement)

        get_logger().info('Condition {}, measurement {}, {}'.format(
            condition, measurement,
            'nominal' if evaluation.nominal else 'not nominal'))

        return evaluation


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


def _fill_parameters(parameters):
    def _process(item):
        if isinstance(item, dict):
            return_value = {}
            for key, value in item.items():
                return_value[key] = _process(value)
            return return_value
        elif isinstance(item, MessageValue):
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


class Publish(Step):
    """Publish a message to a topic."""

    def __init__(self, node, topic, msg_type, duration, rate, parameters,
                 qos_profile_str='system_default',
                 qos_reliability_str='system_default',
                 qos_durability_str='system_default',
                 save_result=False):
        """Publish a message to a topic."""
        Step.__init__(self, save_result)
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


class ServiceCall(Step):
    """Call a ROS Service."""

    def __init__(self, node, service, service_type,
                 parameters=None, save_result=False):
        """Call a ROS Service."""
        Step.__init__(self, save_result)
        if parameters is None:
            parameters = {}
        self.node = node
        self.service = service
        self.service_type = service_type
        self.parameters = parameters

    def execute(self):
        """Call the service."""
        return call_service(
            self.node, self.service, self.service_type,
            _fill_parameters(self.parameters))


class Sleep(Step):
    """Sleep for a specified amount of time."""

    def __init__(self, time, unit='second'):
        """Initialize."""
        Step.__init__(self, False)
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
        time.sleep(float(self.time))
