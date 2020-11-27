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

    def __init__(self, evaluator=None):
        """Initialize."""
        super().__init__(evaluator)

    def evaluate_internal(self, condition, measurement=None):
        """Verify whether measurement matches the lower limit condition."""
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        if condition.lower_limit_is_inclusive:
            nominal = measurement.value >= condition.value
        else:
            nominal = measurement.value > condition.value

        return Evaluation(measurement, condition, nominal)


class UpperLimitEvaluator(Evaluator):
    """Evaluator for the upper limit of numeric conditions."""

    def __init__(self, evaluator=None):
        """Initialize."""
        super().__init__(evaluator)

    def evaluate_internal(self, condition, measurement=None):
        """Verify whether measurement matches the upper limit condition."""
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        if condition.upper_limit_is_inclusive:
            nominal = measurement.value <= condition.value
        else:
            nominal = measurement.value < condition.value

        return Evaluation(measurement, condition, nominal)


class BothLimitsEvaluator(LowerLimitEvaluator, UpperLimitEvaluator):
    """Evaluator for numeric conditions."""

    def __init__(self, evaluator=None):
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
            self, condition.lower_limit, measurement)
        upper_limit_eval = UpperLimitEvaluator.evaluate_internal(
            self, condition.upper_limit, measurement)

        return CompositeEvaluation(
            measurement, condition,
            [lower_limit_eval, upper_limit_eval])


class GreaterThanEvaluator(Evaluator):
    """Greater than predicate."""

    def __init__(self, evaluator=None):
        """Initialize."""
        super().__init__(evaluator)

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

    def __init__(self, evaluator=None):
        """Initialize."""
        super().__init__(evaluator)

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

    def __init__(self, evaluator=None):
        """Initialize."""
        super().__init__(evaluator)

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

    def __init__(self, evaluator=None):
        """Initialize."""
        super().__init__(evaluator)

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

    def __init__(self, evaluator=None):
        """Initialize."""
        super().__init__(evaluator)

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

    def __init__(self, evaluator=None):
        """Initialize."""
        super().__init__(evaluator)

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

    def __init__(self):
        """Initialize."""
        super().__init__()

    def sense_internal(self):
        """Sense internally."""
        pass


class NumericMeasurement(Measurement):
    """Measurement of a numeric value."""

    def __init__(self, value):
        """Initialize."""
        super().__init__(value)

    def __repr__(self):
        """Get a string representation of the numeric measurement."""
        return str(self.value)

    def __lt__(self, other):
        """Compare if value is lesser than other."""
        return self.value < other

    def __le__(self, other):
        """Compare if value is lesser than or equal to other."""
        return self.value <= other

    def __eq__(self, other):
        """Compare if value is equal to other."""
        return self.value == other

    def __ne__(self, other):
        """Compare if value is not equal to other."""
        return self.value != other

    def __gt__(self, other):
        """Compare if value is greater than other."""
        return self.value > other

    def __ge__(self, other):
        """Compare if value is greater than or equal to other."""
        return self.value >= other


Limit = namedtuple('Limit', 'limit is_inclusive')
Range = namedtuple('Range', 'lower upper')


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


class ComparisonCondition(Condition):
    """Condition for a simple comparison."""

    def __init__(
            self,
            value=None,
            retrieve_value=None,
            name=''):
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


class LowerLimitCondition(ComparisonCondition):
    """Condition for a numeric function with just a lower limit."""

    def __init__(
            self,
            lower_limit_is_inclusive,
            lower_limit_value=None,
            retrieve_lower_limit=None,
            name=''):
        """Initialize."""
        super().__init__(lower_limit_value, retrieve_lower_limit, name)

        self.lower_limit_is_inclusive = lower_limit_is_inclusive

    def __repr__(self):
        """Get string representation of the lower limit condition."""
        return '{} lower limit at {}'.format(
            'inclusive' if self.lower_limit_is_inclusive else 'exclusive',
            self.value)


class UpperLimitCondition(ComparisonCondition):
    """Condition for a numeric function with just an upper limit."""

    def __init__(
            self,
            upper_limit_is_inclusive,
            upper_limit_value=None,
            retrieve_upper_limit=None,
            name=''):
        """Initialize."""
        super().__init__(upper_limit_value, retrieve_upper_limit, name)

        self.upper_limit_is_inclusive = upper_limit_is_inclusive

    def __repr__(self):
        """Get a string representation of the upper limit condition."""
        return '{} upper limit at {}'.format(
            'inclusive' if self.upper_limit_is_inclusive else 'exclusive',
            self.value)


class BothLimitsCondition(Condition):
    """Condition with a lower limit and an upper limit."""

    def __init__(
            self,
            lower_limit_is_inclusive,
            upper_limit_is_inclusive,
            lower_limit_value=None,
            upper_limit_value=None,
            retrieve_lower_limit=None,
            retrieve_upper_limit=None,
            name=''):
        """Initialize."""
        super().__init__(
            name=name)
        self.lower_limit = LowerLimitCondition(
            lower_limit_is_inclusive, lower_limit_value,
            retrieve_lower_limit, name)
        self.upper_limit = UpperLimitCondition(
            upper_limit_is_inclusive, upper_limit_value,
            retrieve_upper_limit, name)

    @property
    def value(self):
        """Get the value."""
        lower = Limit(
            self.lower_limit.value, self.lower_limit.lower_limit_is_inclusive)
        upper = Limit(
            self.upper_limit.value, self.upper_limit.upper_limit_is_inclusive)
        return Range(lower, upper)

    def __repr__(self):
        """Get a string representation of the both limits condition."""
        return '{} {} lower limit: {}, {} upper_limit: {})'.format(
            'Both limits' if self.name == '' else self.name,
            'inclusive' if self.lower_limit.lower_limit_is_inclusive
            else 'exclusive',
            self.lower_limit.value,
            'inclusive' if self.upper_limit.upper_limit_is_inclusive
            else 'exclusive',
            self.upper_limit.value)


class GreaterThanCondition(ComparisonCondition):
    """Condition for a numeric value that should be greater than some value."""

    def __repr__(self):
        """Get a string representation of the greater than condition."""
        return f'greater than {self.value}'


class GreaterThanOrEqualToCondition(ComparisonCondition):
    """Condition that value should be greater than or equal to some value."""

    def __repr__(self):
        """Get string representation of the condition."""
        return f'greater than or equal to {self.value}'


class LessThanOrEqualToCondition(ComparisonCondition):
    """Condition that value should be less than or equal to some value."""

    def __repr__(self):
        """Get string representation of the condition."""
        return f'less than or equal to {self.value}'


class LessThanCondition(ComparisonCondition):
    """Condition for a numeric value that should be less than some value."""

    def __repr__(self):
        """Get string representation of the condition."""
        return f'less than {self.value}'


class EqualityCondition(ComparisonCondition):
    """Equality base class."""

    def __init__(
            self,
            value=None,
            retrieve_value=None,
            epsilon=None,
            retrieve_epsilon=None,
            name=''):
        """Initialize."""
        super().__init__(self, value, retrieve_value, name)

        if epsilon is None and retrieve_epsilon is None:
            raise RuntimeError(
                'Either epsilon or retrieve_epsilon has to be specified')

        self._epsilon = epsilon
        self.retrieve_epsilon = retrieve_epsilon

    @property
    def epsilon(self):
        """Get epsilon."""
        return self._epsilon or self.retrieve_epsilon()


class EqualToCondition(EqualityCondition):
    """Condition for a numeric value that should be equal to some value."""

    def __repr__(self):
        """Get string representation of the condition."""
        return f'equal to {self.value} (epsilon: {self.epsilon})'


class NotEqualToCondition(EqualityCondition):
    """Condition for a numeric value that should not be equal to some value."""

    def __repr__(self):
        """Get string representation of the condition."""
        return f'not equal to {self.value} (epsilon: {self.epsilon})'
