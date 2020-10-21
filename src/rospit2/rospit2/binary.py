# Copyright (c) 2020 AIST.
# National Institute of Advanced Industrial Science and Technology.
#
# License: MIT.

"""Binary conditions."""


from abc import ABCMeta, abstractmethod

from .framework import Condition, Evaluation, Evaluator, Measurement, Sensor


def binary_measure(evaluator):
    """Evaluate and wrap the measurement."""
    measurement = evaluator.call_evaluator()
    return measurement_wrapper(measurement)


def measurement_wrapper(measurement):
    """Ensure that a measurement is wrapped in BinaryMeasurement type."""
    if isinstance(measurement, bool):
        return BinaryMeasurement(measurement)
    elif isinstance(measurement, BinaryMeasurement):
        return measurement
    else:
        c = None if measurement is None else measurement.__class__.__name__
        raise TypeError(
            f'measurement should be boolean or BinaryMeasurement, got: {c}')


class BinaryConditionEvaluator(Evaluator):
    """Evaluator for binary values."""

    def __init__(self, sensor):
        """Initialize."""
        Evaluator.__init__(self, sensor)
        self.last_measurement_value = None
        self.last_condition_value = None

    def evaluate_internal(self, condition, measurement=None):
        """Evaluate internally."""
        if measurement is None:
            measurement = binary_measure(self)
        else:
            measurement = measurement_wrapper(measurement)
        assert(isinstance(measurement, BinaryMeasurement))

        if isinstance(measurement, BinaryMeasurement):
            self.last_measurement_value = measurement.value
        elif isinstance(measurement, bool):
            self.last_measurement_value = measurement
        else:
            raise Exception('Measurement is of unexpected type')

        if isinstance(condition, BinaryCondition):
            self.last_condition_value = condition.value
        elif isinstance(condition, bool):
            self.last_condition_value = condition
        else:
            raise Exception('Condition is of unexpected type')

        nominal = self.last_measurement_value == self.last_condition_value
        return Evaluation(measurement, condition, nominal)


class StaticBooleanEvaluator(Evaluator):
    """Evaluator with a static boolean value."""

    def __init__(self, always_true):
        """Initialize."""
        if always_true:
            Evaluator.__init__(self, AlwaysTrueSensor())
        else:
            Evaluator.__init__(self, AlwaysFalseSensor())
        self.always_true = always_true

    def evaluate_internal(self, condition, measurement=None):
        """Evaluate internally."""
        return Evaluation(BinaryMeasurement(self.always_true), condition, True)


class BinarySensor(Sensor):
    """Sensor that returns a binary value."""

    __metaclass__ = ABCMeta

    def __init__(self):
        """Initialize."""
        Sensor.__init__(self)

    @abstractmethod
    def sense_internal(self):
        """Sense internally."""
        pass


class AlwaysTrueSensor(BinarySensor):
    """A sensor that always senses True."""

    def __init__(self):
        """Initialize."""
        BinarySensor.__init__(self)

    def sense_internal(self):
        """Sense True."""
        return True


class AlwaysFalseSensor(BinarySensor):
    """A sensor that always senses False."""

    def __init__(self):
        """Initialize."""
        BinarySensor.__init__(self)

    def sense_internal(self):
        """Sense False."""
        return False


class BinaryMeasurement(Measurement):
    """Measurement of a binary value."""

    def __init__(self, value):
        """Initialize."""
        self.value = value


class BinaryCondition(Condition):
    """Condition that is either True or False."""

    __metaclass__ = ABCMeta

    def __init__(self, value, name=''):
        """Initialize."""
        Condition.__init__(self, value, name)
        self.value = value