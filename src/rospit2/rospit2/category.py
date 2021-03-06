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

"""Evaluators for whether a value is in a specific category."""


from .framework import Condition, Evaluation, Evaluator, Measurement, Sensor


class InCategoryConditionEvaluator(Evaluator):
    """Evaluator for values that should be in a certain category."""

    def __init__(self, sensor):
        """Initialize."""
        super().__init__(sensor)

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

    def __init__(self):
        """Initialize."""
        super().__init__()

    def sense_internal(self):
        """Sense internally."""
        pass


class CategoryMeasurement(Measurement):
    """Measurement of a in category value."""

    def __init__(self, category):
        """Initialize."""
        super().__init__(category)
        self.category = category


class InCategoriesCondition(Condition):
    """Condition that a value is within a list of accepted values."""

    def __init__(
            self,
            categories=None,
            retrieve_categories=None,
            name=''):
        """Initialize."""
        if categories is None:
            if retrieve_categories is None:
                raise RuntimeError(
                    'Categories or retrieve_categories has to be specified')
            super().__init__(name=name)
        else:
            super().__init__(value=categories, name=name)

        self.retrieve_categories = retrieve_categories

    @property
    def value(self):
        """Get the value."""
        return self._value or self.retrieve_categories()
