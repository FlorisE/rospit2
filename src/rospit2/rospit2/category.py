""" Evaluators for whether a value is in a specific category """

from abc import ABCMeta, abstractmethod

from rospit2.framework import Evaluator, Condition, Sensor, Evaluation, Measurement


class InCategoryConditionEvaluator(Evaluator):
    """
    Evaluator for values that should be in a certain category
    """
    def __init__(self, sensor):
        Evaluator.__init__(self, sensor)

    def evaluate_internal(self, condition, measurement=None):
        if measurement is None:
            measurement = self.call_evaluator()
        nominal = measurement.category in condition.categories
        return Evaluation(measurement, condition, nominal)


class CategorySensor(Sensor):
    """
    Sensor that returns a categorical value
    """
    __metaclass__ = ABCMeta

    def __init__(self):
        Sensor.__init__(self)

    @abstractmethod
    def sense_internal(self):
        pass


class CategoryMeasurement(Measurement):
    """
    Measurement of a in category value
    """
    def __init__(self, category):
        self.category = category


class InCategoriesCondition(Condition):
    """
    Condition that a value is within a list of accepted values
    """
    __metaclass__ = ABCMeta

    def __init__(self, categories=None, name=""):
        Condition.__init__(self, name)
        if categories is None:
            categories = set()
        self.categories = categories
