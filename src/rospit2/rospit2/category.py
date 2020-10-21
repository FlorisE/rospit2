# Copyright (c) 2020 AIST.
# National Institute of Advanced Industrial Science and Technology.
#
# License: MIT.

"""Evaluators for whether a value is in a specific category."""

from abc import ABCMeta, abstractmethod

from .framework import Condition, Evaluation, Evaluator, Measurement, Sensor


class InCategoryConditionEvaluator(Evaluator):
    """Evaluator for values that should be in a certain category."""

    def __init__(self, sensor):
        """Initialize."""
        Evaluator.__init__(self, sensor)

    def evaluate_internal(self, condition, measurement=None):
        """
        Evaluate internally.

        If measurement has been provided then evaluate that directly.
        If no measurement has been provided, first call the evaluator.
        """
        if measurement is None:
            measurement = self.call_evaluator()
        nominal = measurement.category in condition.categories
        return Evaluation(measurement, condition, nominal)


class CategorySensor(Sensor):
    """Sensor that returns a categorical value."""

    __metaclass__ = ABCMeta

    def __init__(self):
        """Initialize."""
        Sensor.__init__(self)

    @abstractmethod
    def sense_internal(self):
        """Sense internally."""
        pass


class CategoryMeasurement(Measurement):
    """Measurement of a in category value."""

    def __init__(self, category):
        """Initialize."""
        self.category = category


class InCategoriesCondition(Condition):
    """Condition that a value is within a list of accepted values."""

    __metaclass__ = ABCMeta

    def __init__(self, categories=None, name=''):
        """Initialize."""
        Condition.__init__(self, name)
        if categories is None:
            categories = set()
        self.categories = categories
