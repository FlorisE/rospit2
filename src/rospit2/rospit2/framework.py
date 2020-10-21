# Copyright (c) 2020 AIST.
# National Institute of Advanced Industrial Science and Technology.
#
# Licensed under the MIT License.

"""A framework for running automated tests using external instrumentation."""

import logging
from abc import ABCMeta, abstractmethod
from collections import defaultdict
from time import sleep, strftime


class InvariantFailedException(Exception):
    """Exception thrown when an invariant has failed."""

    def __init__(self):
        """Initialize."""
        super.__init__()


class _TestRunnerState(object):
    """Class to hold the current state of the test runner.

    Grouped in a class instead of being globals as they are not static.
    """

    def __init__(self):
        self.test_suite = None
        self.test_case = None
        self.logger = None


TEST_RUNNER_STATE = _TestRunnerState()


def get_active_test_suite():
    """
    Return the test suite that was last ran.

    >>> get_active_test_suite()
    "Hello"
    """
    return TEST_RUNNER_STATE.test_suite


def get_active_test_case():
    """Return the test case that was last ran."""
    return TEST_RUNNER_STATE.test_case


def get_logger():
    """Return the logger that was passed to the test runner."""
    return TEST_RUNNER_STATE.logger


COLOR_GREEN = '\033[92m'
COLOR_RED = '\033[91m'
RESET_FORMATTING = '\033[0m'


def status_message_from_boolean(value):
    """From a boolean value, return the status message."""
    if value.passed:
        return COLOR_GREEN + 'PASSED' + RESET_FORMATTING
    return COLOR_RED + 'FAILED (%s)' % value.failure_type + RESET_FORMATTING


class Runner(object):
    """Class for running test suites."""

    def __init__(self, logger=None, xml_output_settings=None):
        """Initialize."""
        if logger is None:
            logger = logging.getLogger('rospit default logger')
        self.logger = logger
        TEST_RUNNER_STATE.logger = logger
        self.reports = []
        self.last_suite = None
        self.xml_output_settings = xml_output_settings

    def run_suite(self, test_suite):
        """Run the specified test_suite."""
        self.last_suite = test_suite
        test_suite_report = test_suite.run(self.logger)
        for test_case_report in test_suite_report.test_case_reports:
            status_msg = status_message_from_boolean(test_case_report)
            self.logger.info(
                '%s: %s', test_case_report.test_case.name, status_msg)

        if self.xml_output_settings is not None:
            xml_results_file = open(self.xml_output_settings.export_path, 'w')
            xml_results_file.write(test_suite_report.get_junit_xml())

    def run_all(self):
        """Autodetect test suites and run them."""
        pass


class TestSuite(object):
    """Container for test cases."""

    __metaclass__ = ABCMeta

    def __init__(self, name=''):
        """Initialize."""
        self.name = name
        self.test_cases = []

    def run(self, logger):
        """Run all the test cases in this test suite."""
        TEST_RUNNER_STATE.test_suite = self
        TEST_RUNNER_STATE.logger = logger
        logger.info('Running test suite {}'.format(self.name))
        test_case_reports = [tc.execute(logger) for tc in self.test_cases]
        return TestSuiteReport(self, test_case_reports)


def get_failed_conditions(conditions):
    """Get a comma separated string for the conditions that failed."""
    if conditions is None:
        return ''
    return ', '.join([c.condition.name for c in conditions if not c.nominal])


def all_invariants_nominal(invariants):
    """Check if for each invariant, all of its evaluations are nominal."""
    invariant_evaluations = []
    for invariant, evaluations in invariants.items():
        invariant_evaluations.append(
            Evaluation(
                Measurement(evaluations),
                invariant.condition,
                all_conditions_nominal(evaluations)))
    return all_conditions_nominal(invariant_evaluations)


def all_conditions_nominal(conditions):
    """
    Check whether all conditions are nominal.

    If no conditions are specified, assume nominal.
    """
    if not conditions:
        return True
    return all(c.nominal for c in conditions)


class TestCase(object):
    """A test case with preconditions, invariants and postconditions."""

    __metaclass__ = ABCMeta

    def __init__(self, name='', preconditions=None, invariants=None,
                 postconditions=None, wait_for_preconditions=False,
                 sleep_rate=0.1, depends_on=None):
        """Initialize."""
        self.name = name
        if preconditions is None:
            preconditions = []
        if invariants is None:
            invariants = []
        if postconditions is None:
            postconditions = []
        if depends_on is None:
            depends_on = []
        self.preconditions = preconditions
        self.invariants = invariants
        self.invariants_evaluations = defaultdict(list)
        self.postconditions = postconditions
        self.wait_for_preconditions = wait_for_preconditions
        self.sleep_rate = sleep_rate
        self.depends_on = depends_on
        self.report = None
        self.ran = False
        self.invariant_failed = False

    def set_up(self):
        """Can be overriden to set up the test case."""
        pass

    def tear_down(self):
        """Can be overriden to tear down the test case."""
        pass

    def verify_preconditions(self):
        """Evaluate all preconditions."""
        return [e.evaluate(c) for c, e in self.preconditions]

    def verify_postconditions(self):
        """Evaluate all postconditions."""
        return [e.evaluate(c) for c, e in self.postconditions]

    def all_preconditions_nominal(self, preconditions=None):
        """Check whether all conditions of the test case are nominal."""
        if preconditions is None:
            preconditions = self.verify_preconditions()
        return all_conditions_nominal(preconditions)

    def execute(self, logger):
        """Verify preconditions, run test, verify postconditions."""
        TEST_RUNNER_STATE.logger = logger
        logger.info('Executing test case {}'.format(self.name))

        for dependency in self.depends_on:
            logger.info('Depends on: {}'.format(dependency.name))

        TEST_RUNNER_STATE.test_case = self

        self.set_up()

        not_passed_dependencies = [
            tc for tc in self.depends_on if not tc.ran or not tc.report.passed]

        report = None

        if not_passed_dependencies:
            logger.info(
                'Dependencies have not been met, marking test case as failure')
            report = self.finish([], [], not_passed_dependencies)

        if report is None:
            logger.info('Evaluating preconditions')
            if self.wait_for_preconditions:
                logger.info('Wait for preconditions is enabled')
                preconditions_eval = self.verify_preconditions()
                while not self.all_preconditions_nominal(preconditions_eval):
                    sleep(self.sleep_rate)
                    preconditions_eval = self.verify_preconditions()
            else:
                logger.info('Wait for preconditions is disabled')
                preconditions_eval = self.verify_preconditions()
                if not self.all_preconditions_nominal(preconditions_eval):
                    report = self.finish(preconditions_eval, [], [])

        if report is None:
            logger.info('Running the body of the test')
            self.start_invariant_monitoring()
            try:
                self.run()
            except InvariantFailedException:
                self.stop_invariant_monitoring()
                logger.info('An invariant has failed')
                report = self.finish(preconditions_eval, None)
                return report
            self.stop_invariant_monitoring()
            logger.info('Evaluating postconditions')
            postconditions_eval = self.verify_postconditions()
            report = self.finish(
                preconditions_eval, postconditions_eval)

        return report

    def finish(self, preconditions_evaluation, postconditions_evaluation,
               not_passed_dependencies=None):
        """Finish executing the test case."""
        if not_passed_dependencies is None:
            not_passed_dependencies = []
        self.ran = True
        self.report = TestCaseReport(
            self, preconditions_evaluation, self.invariants_evaluations,
            postconditions_evaluation, not_passed_dependencies)
        self.tear_down()
        return self.report

    def start_invariant_monitoring(self):
        """
        Start invariant monitoring.

        Should append results to self.invariants_evaluation.
        Should be non-blocking, e.g. implemented as a thread for process.
        This should be overridden by a class that wants to use invariants.
        """
        pass

    def stop_invariant_monitoring(self):
        """
        Stop invariant monitoring, for example stopping the thread / process.

        This should be overridden by a class that wants to use invariants.
        """
        pass

    @abstractmethod
    def run(self):
        """Run the test."""
        pass


TEST_CASE_XML_SUCCESS = """<testcase classname='{}' name='{}' />"""

TEST_CASE_XML_FAILURE = """\
<testcase classname='{}' name='{}'>
{}
</testcase>"""

FAILURE = """<failure type='{}'>{}</failure>"""


class TestCaseReport(object):
    """Report which contains the result of executing a test case."""

    def __init__(self, test_case, preconditions=None, invariants=None,
                 postconditions=None, not_passed_dependencies=None):
        """Initialize."""
        if preconditions is None:
            preconditions = []
        if invariants is None:
            invariants = []
        if postconditions is None:
            postconditions = []
        if not_passed_dependencies is None:
            not_passed_dependencies = []
        self.test_case = test_case
        self.preconditions = preconditions
        self.invariants = invariants
        self.postconditions = postconditions
        self.not_passed_dependencies = not_passed_dependencies
        self.preconditions_nominal = all_conditions_nominal(self.preconditions)
        self.invariants_nominal = all_invariants_nominal(self.invariants)
        self.postconditions_nominal = all_conditions_nominal(
            self.postconditions)
        self.passed = (
            self.preconditions_nominal and
            self.invariants_nominal and
            self.postconditions_nominal and
            not self.not_passed_dependencies)
        self.failure_types = []
        if self.not_passed_dependencies:
            self.failure_types.append('dependencies')
        if not self.preconditions_nominal:
            self.failure_types.append('preconditions')
        if not self.invariants_nominal:
            self.failure_types.append('invariants')
        if not self.postconditions_nominal:
            self.failure_types.append('postconditions')
        self.failure_type = None if self.passed else ', '.join(
                self.failure_types)

    def get_failure(self):
        """Get a failure JUnit XML string."""
        if self.passed:
            return ''
        if self.failure_type == 'dependencies':
            failure = ', '.join(
                [tc.name if tc.name != ''
                 else 'UNKNOWN' for tc in self.not_passed_dependencies])
        elif self.failure_type == 'preconditions':
            failure = get_failed_conditions(self.preconditions)
        elif self.failure_type == 'invariants':
            failure = get_failed_conditions(self.invariants)
        else:
            failure = get_failed_conditions(self.postconditions)
        return FAILURE.format(self.failure_type, failure)

    def get_junit_xml(self):
        """Get a JUnit XML summary."""
        if self.passed:
            return TEST_CASE_XML_SUCCESS.format(
                self.test_case.name, self.test_case.name)
        return TEST_CASE_XML_FAILURE.format(
            self.test_case.name, self.test_case.name, self.get_failure())


TEST_SUITE_XML = """\
<testsuite name='{}' tests='{}'>
{}
</testsuite>"""


class TestSuiteReport(object):
    """Report which contains the result of executing a test suite."""

    def __init__(self, test_suite, test_case_reports):
        """Initialize."""
        self.test_suite = test_suite
        self.test_case_reports = test_case_reports

    def get_junit_xml(self, logger=None):
        """Get a JUnit XML summary."""
        result = TEST_SUITE_XML.format(
            self.test_suite.name if self.test_suite is not None else 'UNKNOWN',
            len(self.test_case_reports),
            '\n'.join(
                [report.get_junit_xml() for report in self.test_case_reports]))
        if logger is not None:
            logger.debug(result)
        return result


class XMLOutputSettings(object):
    """Contains settings for outputting an XML test report."""

    def __init__(self, export_path=None):
        """Initialize."""
        if export_path is None:
            export_path = strftime('%Y%m%d%H%M%S.xml')
        self.export_path = export_path


class Evaluation(object):
    """Evaluation produced by an Evaluator."""

    def __init__(self, measurement, condition, nominal):
        """Initialize."""
        if not isinstance(measurement, Measurement):
            raise ValueError(
                'measurement is not of the right type, got: ' +
                measurement.__class__.__name__)
        if not isinstance(condition, Condition):
            raise ValueError(
                'condition is not of the right type, got: ' +
                condition.__class__.__name__)
        self.measurement = measurement
        self.condition = condition
        self.nominal = nominal

    def get_failure(self):
        """Return a failure string."""
        if self.nominal:
            return None
        return self.condition.name

    def expected_actual_string(self, separator=', '):
        """
        Get an evaluation string.

        Contains the expected value and the actual value,
        seperated by the specified separator.
        """
        return str(self.condition.value) + \
            separator + str(self.measurement.value)


class CompositeEvaluation(Evaluation):
    """List of evaluations for a single measurement."""

    def __init__(self, measurement, condition, evaluations):
        """Initialize."""
        Evaluation.__init__(
            self, measurement, condition,
            all(evaluation.nominal for evaluation in evaluations))
        self.evaluations = evaluations


class TimestampEvaluationPair(object):
    """Pair of a timestamp and an evaluation."""

    def __init__(self, timestamp, evaluation):
        """Initialize."""
        self.timestamp = timestamp
        self.evaluation = evaluation


class ConditionEvaluatorPair(object):
    """Pair of a condition and an evaluator."""

    def __init__(self, condition, evaluator):
        """Initialize."""
        self.condition = condition
        self.evaluator = evaluator

    def keys(self):
        """Get keys of the pair."""
        return [0, 1]

    def __getitem__(self, i):
        """Get condition or evaluator."""
        if i == 0:
            return self.condition
        elif i == 1:
            return self.evaluator
        else:
            raise IndexError(
                'Key not supported, use 0 (for condition) or 1 (for evaluator)'
            )


class Condition(object):
    """Abstract base class for conditions."""

    __metaclass__ = ABCMeta

    def __init__(self, value, name=''):
        """Initialize."""
        self.value = value
        self.name = self.__class__.__name__ if name == '' else name

    def __repr__(self):
        """Return a string representation of condition."""
        return '{}({})'.format(self.name, self.value)


class Evaluator(object):
    """Abstract base class for evaluators."""

    __metaclass__ = ABCMeta

    def __init__(self, evaluator):
        """Initialize."""
        self.evaluator = evaluator

    @abstractmethod
    def evaluate_internal(self, condition, measurement=None):
        """
        Evaluate internally.

        Should be implemented by child.
        """
        pass

    def evaluate(self, condition, measurement=None):
        """
        Execute the evaluator.

        Checks the measurement using the condition.
        Should return an instance of Evaluation.
        """
        evaluation = self.evaluate_internal(condition, measurement)
        get_logger().debug('Condition {0}, measurement {1}, {2}'.format(
                           condition,
                           evaluation.measurement,
                           # filled by evaluation in case measurement is None
                           'nominal' if evaluation.nominal else 'not nominal'))
        return evaluation

    def call_evaluator(self):
        """
        Call the evaluator.

        Use sense function if it is a sensor.
        Else, direct invocation if callable.
        """
        if isinstance(self.evaluator, Sensor):
            return self.evaluator.sense()
        elif callable(self.evaluator):
            return self.evaluator()
        else:
            raise Exception('Evaluator is neither a sensor nor callable')

    def call_evaluator_with_data(self, data):
        """Call the evaluator with the specified data."""
        return self.evaluator(data)


class Measurement(object):
    """Contains a measurement."""

    def __init__(self, value):
        """Initialize."""
        self.value = value

    def __repr__(self):
        """Get string representation of the measurement."""
        return '{}({})'.format(self.__class__.__name__, self.value)


class Sensor(object):
    """Abstract base class for sensors."""

    __metaclass__ = ABCMeta

    def __init__(self):
        """Initialize."""
        self.last_sensed = None

    @abstractmethod
    def sense_internal(self):
        """Read the sensor value."""
        pass

    def sense(self):
        """Store the last sensed value."""
        self.last_sensed = self.sense_internal()
        return self.last_sensed
