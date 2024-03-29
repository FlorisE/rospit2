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

"""ROS specific implementation of the testing framework."""


import asyncio
import time
from enum import Enum, auto

from rcl_interfaces.msg import ParameterType

from rosidl_runtime_py.utilities import get_message

from .binary import BinaryMeasurement
from .declarative import DeclarativeTestCase, DeclarativeTestSuite
from .framework import Condition, Evaluation, Evaluator, \
                       Measurement, Measurements
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
from .ros_parameters import store_parameters


def get_numeric_value(value):
    """Get the numeric value of the ParameterValue."""
    if value.type == ParameterType.PARAMETER_INTEGER:
        return value.integer_value
    elif value.type == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    else:
        raise ValueError('Expected integer or double value.')


def get_boolean_value(value):
    """Get the boolean value of the ParameterValue."""
    if value.type == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    else:
        raise ValueError('Expected boolean value.')


def get_string_value(value):
    """Get the string value of the ParameterValue."""
    if value.type == ParameterType.PARAMETER_STRING:
        return value.string_value
    else:
        raise ValueError('Expected string value.')


class StringEqualsCondition(Condition):
    """Condition that a string should equal another string."""

    def __init__(self, value=None, retrieve_value=None, name=''):
        """Initialize."""
        if value is None:
            if retrieve_value is None:
                raise RuntimeError(
                    'Either value or retrieve_value has to be specified')
            super().__init__(name=name)
        else:
            super().__init__(value=value, name=name)

        self.retrieve_value = retrieve_value

    @property
    def value(self):
        """Get the value."""
        return self._value or self.retrieve_value()

    def __repr__(self):
        """Get a string representation of the condition."""
        return f'{self.name}({self.value})'


class SubscriptionManager(object):
    """Manages subscriptions."""

    def __init__(self, test_suite, node, subscribers):
        """Initialize."""
        self.test_suite = test_suite
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
        for topic, msg_type in self.test_suite.msg_value_subscribers:
            if topic not in self.initialized_subscribers:
                msg_module = get_message(msg_type)
                self.initialized_subscribers[topic] = self.get_subscriber(
                    topic, msg_module, self.test_suite.store_message)
            if topic not in self.test_suite.msg_value_subscribers:
                self.test_suite.msg_value_subscribers[topic] = \
                    self.initialized_subscribers[topic]
        for topic, msg_type in self.test_suite.msg_received_subscribers:
            if topic not in self.initialized_subscribers:
                msg_module = get_message(msg_type)
                self.initialized_subscribers[topic] = self.get_subscriber(
                    topic, msg_module, self.test_suite.store_message)
            if topic not in self.test_suite.msg_received_subscribers:
                self.test_suite.msg_received_subscribers[topic] = \
                        self.initialized_subscribers[topic]


class ROSPITSessionEpisode:
    """A test execution specification."""

    def __init__(self,
                 name,
                 path,
                 test_case=None,
                 parameters=None,
                 initialize_service=None,
                 iterate_service=None,
                 iterations=None):
        """Initialize."""
        self.name = name
        self.path = path
        self.test_case = test_case
        self.parameters = parameters
        self.initialize_service = initialize_service
        self.iterate_service = iterate_service
        self.iterations = iterations


class ROSPITSession:
    """A ROSPIT specific test session."""

    def __init__(self, node):
        """Initialize."""
        self.node = node
        self.episodes = []


class ROSTestSuite(DeclarativeTestSuite):
    """A ROS specific test suite."""

    def __init__(
            self, node, subscribers, name='',
            set_up_steps=None, tear_down_steps=None,
            one_time_set_up_steps=None, one_time_tear_down_steps=None):
        """Initialize."""
        super().__init__(
            name, set_up_steps, tear_down_steps,
            one_time_set_up_steps, one_time_tear_down_steps)
        self.node = node
        self.messages = {}
        self.message_received_on = set()
        self.msg_value_subscribers = {}
        self.msg_received_subscribers = {}
        self.subscription_manager = SubscriptionManager(
                self, self.node, subscribers)
        self.stored_parameters = {}

    def run(self, logger, parameters=None):
        """
        Run the test suite.

        Wraps the super method to add subscription management.
        """
        if parameters:
            asyncio.run(
                store_parameters(
                    self.node, parameters, self.stored_parameters))

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
        super().__init__(
            test_suite,
            name=name,
            set_up_steps=set_up_steps,
            preconditions=preconditions,
            wait_for_preconditions=wait_for_preconditions,
            run_steps=run_steps,
            invariants=invariants,
            postconditions=postconditions,
            tear_down_steps=tear_down_steps,
            sleep_rate=sleep_rate,
            depends_on=depends_on)
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
    """Base class for single (last) message evaluation."""

    def __init__(self, node, topic, topic_type, field=None):
        """Initialize the message evaluator."""
        super().__init__(None)
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


class MessagesEvaluatorBase(Evaluator):
    """Base class for multiple messages evaluation."""

    def __init__(self, node, topic, topic_type, field=None):
        """Initialize the message evaluator."""
        super().__init__(None)
        self.received = 0
        self.node = node
        self.topic = topic
        self.topic_type = topic_type
        self.field = field
        self.data = []
        msg_type = get_message(topic_type)
        self.subscriber = self.node.create_subscription(
            msg_type, topic, self.callback, 10)

    def callback(self, data):
        """Fill data and mark as received."""
        self.data.append(get_field_or_message(data, self.field))
        self.received += 1


class MessageReceivedEvaluator(MessageEvaluatorBase):
    """Evaluate whether a message has been received on the topic."""

    def __init__(self, node, topic, topic_type, field=None):
        """Initialize."""
        super().__init__(node, topic, topic_type, field)

    def evaluate_internal(self, c, measurement=None):
        """Internally evaluate the message."""
        if measurement is None:
            measurement = BinaryMeasurement(self.received)
        return Evaluation(
            measurement, c,
            self.received and c.value or not self.received and not c.value)


class Occurrence(Enum):
    """Specifies how something should occur in terms of time."""

    ONCE_AND_ONLY = auto()
    ONLY_ONCE = auto()
    ONCE = auto()
    FIRST = auto()
    LAST = auto()
    ALWAYS = auto()


class MessagesEvaluation(Evaluation):
    """Evaluation of multiple messages."""

    def __init__(self, measurements, condition, nominal, occurrence):
        """Initialize."""
        super().__init__(measurements, condition, nominal)
        self.measurements = measurements
        self.occurrence = occurrence

    def expected_actual_string(self):
        """Get an evaluation string."""
        values = ', '.join([str(value) for value in self.measurements.value])
        if self.occurrence == Occurrence.ONCE:
            return '{} in [{}]'.format(
                self.condition.value,
                values)
        elif self.occurrence == Occurrence.ONLY_ONCE:
            return '{} only once in [{}]'.format(
                self.condition.value,
                values)
        elif self.occurrence == Occurrence.FIRST:
            return '{} first in [{}]'.format(
                self.condition.value,
                values)
        elif self.occurrence == Occurrence.LAST:
            return '{} last in [{}]'.format(
                self.condition.value,
                values)
        elif self.occurrence == Occurrence.ONCE_AND_ONLY:
            return '{} once and only once in [{}]'.format(
                self.condition.value,
                values)
        elif self.occurrence == Occurrence.ALWAYS:
            return '{} always in [{}]'.format(
                self.condition.value,
                values)
        else:
            return f'{self.condition.value}({self.measurements.value})'


class MessageEvaluator(MessageEvaluatorBase):
    """Evaluate the content of a single message."""

    def __init__(
            self, node, topic, topic_type, field=None):
        """Initialize."""
        super().__init__(node, topic, topic_type, field)

    def evaluate_internal(self, condition, measurement=None):
        """Internally evaluate the message."""
        if measurement is None:
            while self.data is None:
                time.sleep(1)
            measurement = self.data

        return Evaluation(measurement, condition, self.data == condition.value)


class MessagesEvaluator(MessagesEvaluatorBase):
    """Evaluate the content of multiple messages."""

    def __init__(
            self, node, topic, topic_type, occurrence,
            negate=False, field=None):
        """Initialize."""
        super().__init__(node, topic, topic_type, field)
        self.occurrence = occurrence
        self.negate = negate

    def evaluate_internal(self, condition, measurement=None):
        """Internally evaluate the message."""
        if measurement is None:
            if not self.data:
                if self.occurrence == Occurrence.ONCE or \
                   self.occurrence == Occurrence.ONCE_AND_ONLY or \
                   self.occurrence == Occurrence.ONLY_ONCE or \
                   self.occurrence == Occurrence.FIRST or \
                   self.occurrence == Occurrence.LAST:
                    return MessagesEvaluation(
                        Measurements([]),
                        condition,
                        False if not self.negate else True,
                        self.occurrence)
                elif self.occurrence == Occurrence.ALWAYS:
                    return MessagesEvaluation(
                        Measurements([]),
                        condition,
                        True,
                        self.occurrence)
                else:
                    raise ValueError('Occurrence is of unknown type')
            measurements = Measurements(self.data)

        if self.occurrence == Occurrence.ONCE:
            e = condition.value in measurements.value
        elif self.occurrence == Occurrence.ONLY_ONCE:
            e = measurements.value.count(condition.value) == 1
        elif self.occurrence == Occurrence.FIRST:
            e = len(measurements.value) > 0 and \
                measurements.value[0] == condition.value
        elif self.occurrence == Occurrence.LAST:
            e = len(measurements.value) > 0 and \
                measurements.value[-1] == condition.value
        elif self.occurrence == Occurrence.ONCE_AND_ONLY:
            e = len(measurements.value) == 1 and \
                measurements.value[0] == condition.value
        elif self.occurrence == Occurrence.ALWAYS:
            e = all(condition.value == value for value in measurements.value)
        return MessagesEvaluation(
            measurements,
            condition,
            e and not self.negate or not e and self.negate,
            self.occurrence)


class ExecutionReturnedEvaluator(Evaluator):
    """Evaluator for whether execution has returned."""

    def __init__(self, test_case, field=None):
        """Initialize."""
        super().__init__(None)
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


class UnableToEvaluateMessageException(Exception):
    """Exception thrown when unable to evaluate a message."""

    pass


class NumericMessagesEvaluator(MessagesEvaluatorBase):
    """Evaluator for numeric messages."""

    def __init__(
            self, node, topic, topic_type, occurrence,
            negate=False, field=None):
        """Initialize."""
        super().__init__(node, topic, topic_type, field)
        self.occurrence = occurrence
        self.negate = negate

    def evaluate_internal(self, condition, measurements=None):
        """Internally evaluate the numeric messages."""
        if measurements is None:
            if not self.data:
                if self.occurrence == Occurrence.ONCE or \
                   self.occurrence == Occurrence.ONCE_AND_ONLY or \
                   self.occurrence == Occurrence.ONLY_ONCE or \
                   self.occurrence == Occurrence.FIRST or \
                   self.occurrence == Occurrence.LAST:
                    return MessagesEvaluation(
                        Measurements([]),
                        condition,
                        False if not self.negate else True,
                        self.occurrence)
                elif self.occurrence == Occurrence.ALWAYS:
                    return MessagesEvaluator(
                        Measurements([]),
                        condition,
                        True,
                        self.occurrence)
                else:
                    raise ValueError('Occurrence is of unknown type')
            else:
                measurements = Measurements(
                    [NumericMeasurement(datum) for datum in self.data])

        evaluator = NumericMessageEvaluator(
            self.node, self.topic, self.topic_type, self.field)
        overall_evaluation = None
        if self.occurrence == Occurrence.ONCE:
            for measurement in measurements.value:
                evaluation = evaluator.evaluate(condition, measurement)
                if evaluation.nominal:
                    overall_evaluation = MessagesEvaluation(
                        measurements, condition,
                        evaluation.nominal, self.occurrence)
                    break
        elif self.occurrence == Occurrence.ONLY_ONCE:
            for measurement in measurements.value:
                evaluation = evaluator.evaluate(condition, measurement)
                if evaluation.nominal:
                    if overall_evaluation:
                        overall_evaluation = MessagesEvaluation(
                            measurements, condition,
                            not evaluation.nominal, self.occurrence)
                        break
                    else:
                        overall_evaluation = MessagesEvaluation(
                            measurements, condition,
                            evaluation.nominal, self.occurrence)
        elif self.occurrence == Occurrence.FIRST:
            evaluation = evaluator.evaluate(condition, measurements.value[0])
            overall_evaluation = MessagesEvaluation(
                measurements, condition,
                evaluation.nominal, self.occurrence)
        elif self.occurrence == Occurrence.LAST:
            evaluation = evaluator.evaluate(condition, measurements.value[-1])
            overall_evaluation = MessagesEvaluation(
                measurements, condition,
                evaluation.nominal, self.occurrence)
        elif self.occurrence == Occurrence.ONCE_AND_ONLY:
            evaluation = len(measurements.value) == 1 and \
                    evaluator.evaluate(condition, measurements.value[0])
            overall_evaluation = MessagesEvaluation(
                measurements, condition,
                evaluation.nominal, self.occurrence)
        elif self.occurrence == Occurrence.ALWAYS:
            for measurement in measurements.value:
                evaluation = evaluator.evaluate(condition, measurement)
                if not evaluation.nominal:
                    overall_evaluation = MessagesEvaluation(
                        measurements, condition,
                        evaluation.nominal, self.occurrence)
                    break
            overall_evaluation = MessagesEvaluation(
                measurements, condition, True, self.occurrence)

        if not overall_evaluation:
            raise UnableToEvaluateMessageException()

        return overall_evaluation


class NumericMessageEvaluator(MessageEvaluatorBase):
    """Evaluator for a single numeric message."""

    def __init__(
            self, node, topic, topic_type, field=None):
        """Initialize."""
        super().__init__(node, topic, topic_type, field)

    def evaluate_internal(self, condition, measurement=None):
        """Internally evaluate the numeric message."""
        if measurement is None:
            while self.data is None:
                time.sleep(1)
            measurement = self.data
        if measurement is not NumericMeasurement:
            measurement = NumericMeasurement(measurement)

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

        evaluator = evaluator_type()

        evaluation = evaluator.evaluate(condition, measurement)

        return Evaluation(measurement, condition, evaluation.nominal)
