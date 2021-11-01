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

"""Declarative framework for specifying tests."""


from .framework import InvariantFailedException, TestCase, TestSuite


class DeclarativeTestSuite(TestSuite):
    """
    A test suite that declares how it should be executed.

    Can be used in for example scripting environments for creating test suites.
    """

    def __init__(
            self, name='', set_up_steps=None, tear_down_steps=None,
            one_time_set_up_steps=None, one_time_tear_down_steps=None):
        """Initialize."""
        super().__init__(name)
        if set_up_steps is None:
            set_up_steps = []
        if tear_down_steps is None:
            tear_down_steps = []
        if one_time_set_up_steps is None:
            one_time_set_up_steps = []
        if one_time_tear_down_steps is None:
            one_time_tear_down_steps = []
        self.set_up_steps = set_up_steps
        self.tear_down_steps = tear_down_steps
        self.one_time_set_up_steps = one_time_set_up_steps
        self.one_time_tear_down_steps = one_time_tear_down_steps

    def one_time_set_up(self):
        """Set up test suite, executed once per test suite execution."""
        for step in self.one_time_set_up_steps:
            step.execute()

    def set_up(self):
        """Set up test suite, re-executed for each test case execution."""
        for step in self.set_up_steps:
            step.execute()

    def _run(self, logger):
        """Run implementation, adds set up steps/tear down steps."""
        self.one_time_set_up()
        results = super()._run(logger)
        self.one_time_tear_down()
        return results

    def tear_down(self):
        """Tear down test suite, re-executed for each test case execution."""
        for step in self.tear_down_steps:
            step.execute()

    def one_time_tear_down(self):
        """Tear down test suite, executed once per test suite execution."""
        for step in self.one_time_tear_down_steps:
            step.execute()

    def clean_up(self):
        """Clean up the execution."""
        for step in self.one_time_set_up_steps + self.one_time_tear_down_steps:
            step.clean_up()


class DeclarativeTestCase(TestCase):
    """
    A test case that completely declares how it should be executed.

    Can be used in for example scripting environments for creating test cases.
    """

    def __init__(
            self, test_suite, name='',
            set_up_steps=None, run_steps=None, tear_down_steps=None,
            preconditions=None, invariants=None,
            postconditions=None, wait_for_preconditions=False,
            sleep_rate=0.1, depends_on=None):
        """Initialize."""
        super().__init__(
            test_suite, name, preconditions, invariants, postconditions,
            wait_for_preconditions, sleep_rate, depends_on)
        if set_up_steps is None:
            set_up_steps = []
        if tear_down_steps is None:
            tear_down_steps = []
        if run_steps is None:
            run_steps = []
        self.set_up_steps = set_up_steps
        self.run_steps = run_steps
        self.tear_down_steps = tear_down_steps
        self.execution_result = None

    def set_up(self):
        """Set up the test case."""
        for step in self.set_up_steps:
            step.execute()

    def run(self):
        """Run the test case."""
        for step in self.run_steps:
            if self.invariant_failed:
                raise InvariantFailedException()
            result = step.execute()
            if step.save_result:
                self.execution_result = result

    def tear_down(self):
        """Tear down the test case."""
        for step in self.tear_down_steps:
            step.execute()

    def clean_up(self):
        """Clean up all executed steps."""
        for step in \
                self.test_suite.set_up_steps + \
                self.set_up_steps + \
                self.run_steps + \
                self.tear_down_steps + \
                self.test_suite.tear_down_steps:
            step.clean_up()


class Step(object):
    """Step that can be used in the set up, run or tear down."""

    def __init__(self, save_result=False):
        """Initialize."""
        self.save_result = save_result
        self.step_type = self.__class__.__name__

    def execute(self):
        """Execute the step."""
        pass

    def clean_up(self):
        """Clean up the step."""
        pass

    def __call__(self):
        """Call the service."""
        return self.execute()


class DummyStep(Step):
    """A step which does nothing, mostly for testing purposes."""

    def __init__(self, save_result=False):
        """Initialize."""
        super().__init__(save_result)

    def execute(self):
        """Execute the step."""
        pass
