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

"""Numeric evaluation, e.g. a function with limits."""

from abc import ABCMeta, abstractmethod
from collections import namedtuple

from .framework import CompositeEvaluation, Condition, Evaluation, Evaluator, \
                       Measurement, Sensor


def numeric_measure(evaluator):
    """Evaluate and wrap the measurement."""
    measurement = evaluator.call_evaluator()
    return measurement_wrapper(measurement)


def measurement_wrapper(measurement):
    """Wrap a measurement with the appropriate wrapper if necessary."""
    if isinstance(measurement, int) or isinstance(measurement, float):
        return NumericMeasurement(measurement)
    elif isinstance(measurement, NumericMeasurement):
        return measurement
    else:
        format_content = measurement or measurement.__class__.__name__

        raise TypeError(
            f'measurement should either be float, int or NumericMeasurement, \
                    but got: {format_content}')


class LowerLimitEvaluator(Evaluator):
    """Evaluator for the lower limit of numeric conditions."""

    def __init__(self, evaluator):
        """Initialize."""
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        """Verify whether measurement matches the lower limit condition."""
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        if condition.lower_limit_is_inclusive:
            nominal = measurement.value >= condition.lower_limit
        else:
            nominal = measurement.value > condition.lower_limit

        return Evaluation(measurement, condition, nominal)


class UpperLimitEvaluator(Evaluator):
    """Evaluator for the upper limit of numeric conditions."""

    def __init__(self, evaluator):
        """Initialize."""
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        """Verify whether measurement matches the upper limit condition."""
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        if condition.upper_limit_is_inclusive:
            nominal = measurement.value <= condition.upper_limit
        else:
            nominal = measurement.value < condition.upper_limit

        return Evaluation(measurement, condition, nominal)


class BothLimitsEvaluator(LowerLimitEvaluator, UpperLimitEvaluator):
    """Evaluator for numeric conditions."""

    def __init__(self, evaluator):
        """Initialize."""
        LowerLimitEvaluator.__init__(self, evaluator)
        UpperLimitEvaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        """
        Evaluate internally.

        Verify whether measurement matches the lower limit and
        upper limit conditions.
        """
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        lower_limit_eval = LowerLimitEvaluator.evaluate_internal(
            self, condition, measurement)
        upper_limit_eval = UpperLimitEvaluator.evaluate_internal(
            self, condition, measurement)

        return CompositeEvaluation(
            measurement, condition,
            [lower_limit_eval, upper_limit_eval])


class GreaterThanEvaluator(Evaluator):
    """Greater than predicate."""

    def __init__(self, evaluator):
        """Initialize."""
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        """
        Evaluate internally.

        If measurement has been provided then evaluate that directly.
        If no measurement has been provided, first call the evaluator.
        """
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        return Evaluation(
            measurement, condition, measurement.value > condition.value)


class GreaterThanOrEqualToEvaluator(Evaluator):
    """Greater than or equal to predicate."""

    def __init__(self, evaluator):
        """Initialize."""
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        """
        Evaluate internally.

        If measurement has been provided then evaluate that directly.
        If no measurement has been provided, first call the evaluator.
        """
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        return Evaluation(
            measurement, condition, measurement.value >= condition.value)


class EqualToEvaluator(Evaluator):
    """Equal to predicate."""

    def __init__(self, evaluator):
        """Initialize."""
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        """
        Evaluate internally.

        If measurement has been provided then evaluate that directly.
        If no measurement has been provided, first call the evaluator.
        """
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        return Evaluation(
            measurement, condition,
            measurement.value >= condition.value - self.epsilon and
            measurement.value <= condition.value + self.epsilon)


class NotEqualToEvaluator(Evaluator):
    """Not equal to predicate."""

    def __init__(self, evaluator):
        """Initialize."""
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        """
        Evaluate internally.

        If measurement has been provided then evaluate that directly.
        If no measurement has been provided, first call the evaluator.
        """
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        return Evaluation(
            measurement, condition, measurement.value != condition.value)


class LessThanOrEqualToEvaluator(Evaluator):
    """Less than or equal to predicate."""

    def __init__(self, evaluator):
        """Initialize."""
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        """
        Evaluate internally.

        If measurement has been provided then evaluate that directly.
        If no measurement has been provided, first call the evaluator.
        """
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        return Evaluation(
            measurement, condition, measurement.value <= condition.value)


class LessThanEvaluator(Evaluator):
    """Less than predicate."""

    def __init__(self, evaluator):
        """Initialize."""
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        """
        Evaluate internally.

        If measurement has been provided then evaluate that directly.
        If no measurement has been provided, first call the evaluator.
        """
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        return Evaluation(measurement, condition,
                          measurement.value < condition.value)


class NumericSensor(Sensor):
    """Sensor that reads a numeric value."""

    __metaclass__ = ABCMeta

    def __init__(self):
        """Initialize."""
        Sensor.__init__(self)

    @abstractmethod
    def sense_internal(self):
        """Sense internally."""
        pass


class NumericMeasurement(Measurement):
    """Measurement of a numeric value."""

    def __init__(self, value):
        """Initialize."""
        Measurement.__init__(self, value)

    def __repr__(self):
        """Get a string representation of the numeric measurement."""
        return 'NumericMeasurement({})'.format(self.value)


Limit = namedtuple('Limit', 'limit is_inclusive')


def get_inclusive_limit(value):
    """
    Get an inclusive limit at the specified value.

    >>> limit = get_inclusive_limit(-2)
    >>> limit.limit
    -2
    >>> limit.is_inclusive
    True
    """
    return Limit(value, True)


def get_exclusive_limit(value):
    """
    Get an exclusive limit at the specified value.

    >>> limit = get_exlusive_limit(-2)
    >>> limit.limit
    -2
    >>> limit.is_inclusive
    False
    """
    return Limit(value, False)


def try_get_limit(limit):
    """Try converting a limit to a tuple."""
    limit_type = type(limit)
    if limit_type is float or limit_type is int:
        return (limit, True)
    elif limit_type is Limit:
        return (limit.limit, limit.is_inclusive)
    else:
        raise TypeError(
            'limit needs to be either an instance of Limit, an int or a float')


class LowerLimitCondition(Condition):
    """Condition for a numeric function with just a lower limit."""

    __metaclass__ = ABCMeta

    def __init__(
            self, lower_limit, name=''):
        """Initialize."""
        Condition.__init__(self, lower_limit, name)
        self.lower_limit, self.lower_limit_is_inclusive = try_get_limit(
            lower_limit)
        self.evaluator_type = LowerLimitEvaluator

    def __repr__(self):
        """Get string representation of the lower limit condition."""
        return '{} lower limit at {}'.format(
            'inclusive' if self.lower_limit_is_inclusive else 'exclusive',
            self.lower_limit)


class UpperLimitCondition(Condition):
    """Condition for a numeric function with just an upper limit."""

    __metaclass__ = ABCMeta

    def __init__(
            self, upper_limit, name=''):
        """Initialize."""
        Condition.__init__(self, upper_limit, name)
        self.upper_limit, self.upper_limit_is_inclusive = try_get_limit(
            upper_limit)

    def __repr__(self):
        """Get a string representation of the upper limit condition."""
        return '{} upper limit at {}'.format(
            'inclusive' if self.upper_limit_is_inclusive else 'exclusive',
            self.upper_limit)


class BothLimitsCondition(LowerLimitCondition, UpperLimitCondition):
    """Condition with a lower limit and an upper limit."""

    __metaclass__ = ABCMeta

    def __init__(
            self, lower_limit, upper_limit, name=''):
        """Initialize."""
        LowerLimitCondition.__init__(self, lower_limit, name)
        UpperLimitCondition.__init__(self, upper_limit, name)

    def __repr__(self):
        """Get a string representation of the both limits condition."""
        return '{} {} lower limit: {}, {} upper_limit: {})'.format(
                'Both limits' if self.name == '' else self.name,
                'inclusive' if self.lower_limit_is_inclusive else 'exclusive',
                self.lower_limit,
                'inclusive' if self.upper_limit_is_inclusive else 'exclusive',
                self.upper_limit)


class GreaterThanCondition(Condition):
    """Condition for a numeric value that should be greater than some value."""

    def __init__(self, value, name=''):
        """Initialize."""
        Condition.__init__(self, value, name)
        self.greater_than = value

    def __repr__(self):
        """Get a string representation of the greater than condition."""
        return 'greater than {}'.format(self.greater_than)


class GreaterThanOrEqualToCondition(Condition):
    """Condition that value should be greater than or equal to some value."""

    def __init__(self, value, name=''):
        """Initialize."""
        Condition.__init__(self, value, name)
        self.greater_than_or_equal_to = value

    def __repr__(self):
        """Get string representation of the condition."""
        return 'greater than or equal to {}'.format(
            self.greater_than_or_equal_to)


class EqualToCondition(Condition):
    """Condition for a numeric value that should be equal to some value."""

    def __init__(self, value, epsilon, name=''):
        """Initialize."""
        Condition.__init__(self, value, name)
        self.equal_to = value
        self.epsilon = epsilon

    def __repr__(self):
        """Get string representation of the condition."""
        return 'equal to {}'.format(self.equal_to)


class NotEqualToCondition(Condition):
    """Condition for a numeric value that should not be equal to some value."""

    def __init__(self, value, name=''):
        """Initialize."""
        Condition.__init__(self, value, name)
        self.not_equal_to = value

    def __repr__(self):
        """Get string representation of the condition."""
        return 'not equal to {}'.format(self.not_equal_to)


class LessThanOrEqualToCondition(Condition):
    """Condition that value should be less than or equal to some value."""

    def __init__(self, value, name=''):
        """Initialize."""
        Condition.__init__(self, value, name)
        self.less_than_or_equal_to = value

    def __repr__(self):
        """Get string representation of the condition."""
        return 'less than or equal to {}'.format(self.less_than_or_equal_to)


class LessThanCondition(Condition):
    """Condition for a numeric value that should be less than some value."""

    def __init__(self, value, name=''):
        """Initialize."""
        Condition.__init__(self, value, name)
        self.less_than = value

    def __repr__(self):
        """Get string representation of the condition."""
        return 'less than {}'.format(self.less_than)
