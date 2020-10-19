from abc import ABCMeta, abstractmethod
from rospit2.framework import TestCase, InvariantFailedException


class DeclarativeTestCase(TestCase):
    """
    A test case that completely declares how it should be executed.
    Can be used in for example scripting environments for creating test cases.
    """
    def __init__(self, run_steps=None, set_up_steps=None, tear_down_steps=None,
                 name="", preconditions=None, invariants=None,
                 postconditions=None, wait_for_preconditions=False,
                 sleep_rate=0.1, depends_on=None):
        TestCase.__init__(
            self, name, preconditions, invariants, postconditions,
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
        """Set up the test case"""
        for step in self.set_up_steps:
            step.execute()

    def run(self):
        """Runs the test case"""
        for step in self.run_steps:
            if self.invariant_failed:
                raise InvariantFailedException()
            result = step.execute()
            if step.save_result:
                self.execution_result = result

    def tear_down(self):
        """Tears down the test case"""
        for step in self.tear_down_steps:
            step.execute()


class Step(object):
    """
    A step that can be used in the set up, run or tear down, when using the declarative API
    """
    __metaclass__ = ABCMeta

    def __init__(self, save_result=False):
        self.save_result = save_result

    @abstractmethod
    def execute(self):
        """Executes the step"""
        pass

    def __call__(self):
        return self.execute()


class DummyStep(Step):
    """
    A step which does nothing, mostly for testing purposes
    """
    def __init__(self, save_result=False):
        Step.__init__(self, save_result)

    def execute(self):
        """Executes the step"""
        pass
