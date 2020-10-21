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
