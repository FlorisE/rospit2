""" Numeric evaluation, e.g. a function with limits """

from collections.abc import ABCMeta, abstractmethod
from collections import namedtuple

from .framework import Evaluator, Evaluation, CompositeEvaluation, Sensor, \
                              Measurement, Condition, get_active_test_suite, get_active_test_case


def numeric_measure(evaluator):
    """Evaluates and wraps the measurement"""
    measurement = evaluator.call_evaluator()
    return measurement_wrapper(measurement)


def measurement_wrapper(measurement):
    if isinstance(measurement, int) or isinstance(measurement, float):
        return NumericMeasurement(measurement)
    elif isinstance(measurement, NumericMeasurement):
        return measurement
    else:
        estr = "measurement should either be a float, an int or a NumericMeasurement, but got: {}"
        raise TypeError(
            estr.format(None if measurement is None else measurement.__class__.__name__))


class LowerLimitEvaluator(Evaluator):
    """
    Evaluator for the lower limit of numeric conditions
    """
    def __init__(self, evaluator):
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        """
        Verifies whether measurement matches the lower limit condition
        """
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
    """
    Evaluator for the upper limit of numeric conditions
    """
    def __init__(self, evaluator):
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        """
        Verifies whether measurement matches the upper limit condition
        """
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
    """
    Evaluator for numeric conditions
    """
    def __init__(self, evaluator):
        LowerLimitEvaluator.__init__(self, evaluator)
        UpperLimitEvaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        """
        Verifies whether measurement matches the lower limit and
        upper limit conditions
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
    def __init__(self, evaluator):
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        return Evaluation(measurement, condition, measurement.value > condition.value)


class GreaterThanOrEqualToEvaluator(Evaluator):
    def __init__(self, evaluator):
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        return Evaluation(measurement, condition, measurement.value >= condition.value)


class EqualToEvaluator(Evaluator):
    def __init__(self, evaluator):
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        return Evaluation(measurement, condition, measurement.value == condition.value)


class NotEqualToEvaluator(Evaluator):
    def __init__(self, evaluator):
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        return Evaluation(measurement, condition, measurement.value != condition.value)


class LessThanOrEqualToEvaluator(Evaluator):
    def __init__(self, evaluator):
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        return Evaluation(measurement, condition, measurement.value <= condition.value)


class LessThanEvaluator(Evaluator):
    def __init__(self, evaluator):
        Evaluator.__init__(self, evaluator)

    def evaluate_internal(self, condition, measurement=None):
        if measurement is None:
            measurement = numeric_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, NumericMeasurement))

        return Evaluation(measurement, condition,
                          measurement.value < condition.value)


class NumericSensor(Sensor):
    """
    Sensor that reads a numeric value
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        Sensor.__init__(self)

    @abstractmethod
    def sense_internal(self):
        pass


class NumericMeasurement(Measurement):
    """
    Measurement of a numeric value
    """
    def __init__(self, value):
        Measurement.__init__(self, value)

    def __repr__(self):
        return "NumericMeasurement({})".format(self.value)


Limit = namedtuple("Limit", "limit is_inclusive")


def get_inclusive_limit(value):
    """
    Gets an inclusive limit at the specified value
    >>> limit = get_inclusive_limit(-2)
    >>> limit.limit
    -2
    >>> limit.is_inclusive
    True
    """
    return Limit(value, True)


def get_exclusive_limit(value):
    """
    Gets an exclusive limit at the specified value
    >>> limit = get_exlusive_limit(-2)
    >>> limit.limit
    -2
    >>> limit.is_inclusive
    False
    """
    return Limit(value, False)


def try_get_limit(limit):
    limit_type = type(limit)
    if limit_type is float or limit_type is int:
        return (limit, True)
    elif limit_type is Limit:
        return (limit.limit, limit.is_inclusive)
    else:
        raise TypeError("limit needs to be either an instance of Limit, an int or a float")


class LowerLimitCondition(Condition):
    """
    A condition for a numeric function with just a lower limit
    """
    __metaclass__ = ABCMeta

    def __init__(
            self, lower_limit, name=""):
        Condition.__init__(self, lower_limit, name)
        self.lower_limit, self.lower_limit_is_inclusive = try_get_limit(lower_limit)
        self.evaluator_type = LowerLimitEvaluator

    def __repr__(self):
        return "{} lower limit at {}".format(
            "inclusive" if self.lower_limit_is_inclusive
                        else "exclusive",
            self.lower_limit)


class UpperLimitCondition(Condition):
    """
    A condition for a numeric function with just an upper limit
    """
    __metaclass__ = ABCMeta

    def __init__(
            self, upper_limit, name=""):
        Condition.__init__(self, upper_limit, name)
        self.upper_limit, self.upper_limit_is_inclusive = try_get_limit(upper_limit)

    def __repr__(self):
        return "{} upper limit at {}".format(
            "inclusive" if self.upper_limit_is_inclusive
                        else "exclusive", self.upper_limit)


class BothLimitsCondition(LowerLimitCondition, UpperLimitCondition):
    """
    A condition for a numeric function with a lower limit and an upper limit
    """
    __metaclass__ = ABCMeta

    def __init__(
            self, lower_limit, upper_limit, name=""):
        LowerLimitCondition.__init__(self, lower_limit, name)
        UpperLimitCondition.__init__(self, upper_limit, name)

    def __repr__(self):
        return "{} {} lower limit: {}, {} upper_limit: {})".format(
                "Both limits" if self.name == "" else self.name,
                "inclusive" if self.lower_limit_is_inclusive
                            else "exclusive", self.lower_limit,
                "inclusive" if self.upper_limit_is_inclusive
                            else "exclusive", self.upper_limit)


class GreaterThanCondition(Condition):
    """
    A condition for a numeric value that should be greater than some value
    """
    def __init__(self, value, name=""):
        Condition.__init__(self, value, name)
        self.greater_than = value

    def __repr__(self):
        return "greater than {}".format(self.greater_than)


class GreaterThanOrEqualToCondition(Condition):
    """
    A condition for a numeric value that should be greater than or equal
    to some value
    """
    def __init__(self, value, name=""):
        Condition.__init__(self, value, name)
        self.greater_than_or_equal_to = value

    def __repr__(self):
        return "greater than or equal to {}".format(self.greater_than_or_equal_to)


class EqualToCondition(Condition):
    """
    A condition for a numeric value that should be equal to some value
    """
    def __init__(self, value, name=""):
        Condition.__init__(self, value, name)
        self.equal_to = value

    def __repr__(self):
        return "equal to {}".format(self.equal_to)


class NotEqualToCondition(Condition):
    """
    A condition for a numeric value that should not be equal to some value
    """
    def __init__(self, value, name=""):
        Condition.__init__(self, value, name)
        self.not_equal_to = value

    def __repr__(self):
        return "not equal to {}".format(self.not_equal_to)


class LessThanOrEqualToCondition(Condition):
    """
    A condition for a numeric value that should be less than or equal
    to some value
    """
    def __init__(self, value, name=""):
        Condition.__init__(self, value, name)
        self.less_than_or_equal_to = value

    def __repr__(self):
        return "less than or equal to {}".format(self.less_than_or_equal_to)


class LessThanCondition(Condition):
    """
    A condition for a numeric value that should be less than some value
    """
    def __init__(self, value, name=""):
        Condition.__init__(self, value, name)
        self.less_than = value

    def __repr__(self):
        return "less than {}".format(self.less_than)
