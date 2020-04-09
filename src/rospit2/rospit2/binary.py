""" Binary conditions """

from abc import ABCMeta, abstractmethod

from rospit2.framework import Evaluator, Evaluation, Measurement, Condition, Sensor


def binary_measure(evaluator):
    """Evaluates and wraps the measurement"""
    measurement = evaluator.call_evaluator()
    return measurement_wrapper(measurement)


def measurement_wrapper(measurement):
    """Ensures that a measurement is properly wrapped in its BinaryMeasurement type"""
    if isinstance(measurement, bool):
        return BinaryMeasurement(measurement)
    elif isinstance(measurement, BinaryMeasurement):
        return measurement
    else:
        estr = "measurement should be boolean or a BinaryMeasurement, but got: {}"
        raise TypeError(
            estr.format(None if measurement is None else measurement.__class__.__name__))


class BinaryConditionEvaluator(Evaluator):
    """
    Evaluator for binary values
    """
    def __init__(self, sensor):
        Evaluator.__init__(self, sensor)
        self.last_measurement_value = None
        self.last_condition_value = None

    def evaluate_internal(self, condition, measurement=None):
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
            raise Exception("Measurement is of unexpected type")

        if isinstance(condition, BinaryCondition):
            self.last_condition_value = condition.value
        elif isinstance(condition, bool):
            self.last_condition_value = condition
        else:
            raise Exception("Condition is of unexpected type")

        nominal = self.last_measurement_value == self.last_condition_value
        return Evaluation(measurement, condition, nominal)



class StaticBooleanEvaluator(Evaluator):
    def __init__(self, always_true):
        if always_true:
            Evaluator.__init__(self, AlwaysTrueSensor())
        else:
            Evaluator.__init__(self, AlwaysFalseSensor())
        self.always_true = always_true

    def evaluate_internal(self, condition, measurement=None):
        return Evaluation(BinaryMeasurement(self.always_true), condition, True)


class BinarySensor(Sensor):
    """
    Sensor that returns a binary value
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        Sensor.__init__(self)

    @abstractmethod
    def sense_internal(self):
        pass


class AlwaysTrueSensor(BinarySensor):
    def __init__(self):
        BinarySensor.__init__(self)

    def sense_internal(self):
        return True


class AlwaysFalseSensor(BinarySensor):
    def __init__(self):
        BinarySensor.__init__(self)

    def sense_internal(self):
        return False


class BinaryMeasurement(Measurement):
    """
    Measurement of a binary value
    """
    def __init__(self, value):
        self.value = value


class BinaryCondition(Condition):
    """
    Condition that is either True or False
    """
    __metaclass__ = ABCMeta

    def __init__(self, value, name=""):
        Condition.__init__(self, value, name)
        self.value = value
