# Copyright (c) 2020 AIST.
# National Institute of Advanced Industrial Science and Technology.
#
# Licensed under the MIT License.

"""Helper functions for running tests."""


from rospit_msgs.msg import Condition as ConditionMessage, \
                            ConditionEvaluationPair as CEPMessage, \
                            Evaluation as EvaluationMessage, \
                            TestCase as TestCaseMessage, \
                            TestCaseReport as TestCaseReportMessage, \
                            TestSuiteReport as TestSuiteReportMessage


def map_evaluation(evaluation):
    """Map evaluation from the framework to ROS ConditionEvaluationPair."""
    condition_msg = ConditionMessage(name=evaluation.condition.name)
    evaluation_msg = EvaluationMessage(
        nominal=evaluation.nominal,
        payload=evaluation.expected_actual_string())
    return CEPMessage(condition=condition_msg, evaluation=evaluation_msg)


def map_test_case_report(report):
    """Map a test case report from the framework to a ROS message."""
    tc = TestCaseMessage(name=report.test_case.name)
    p_c = [map_evaluation(evaluation) for evaluation in report.preconditions]
    iv = []
    for _, evaluations in report.invariants.items():
        for evaluation in evaluations:
            iv.append(map_evaluation(evaluation))
    post_c = [map_evaluation(ev) for ev in report.postconditions]
    return TestCaseReportMessage(
        test_case=tc, preconditions=p_c, invariants=iv, postconditions=post_c)


def map_test_suite_report(report):
    """Map a report from the test framework to a ROS message."""
    reports = [map_test_case_report(tcr) for tcr in report.test_case_reports]
    return TestSuiteReportMessage(
        test_suite_name=report.test_suite.name, test_case_reports=reports)
