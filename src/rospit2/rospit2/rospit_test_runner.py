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

"""Script that launches PIT nodes and services."""

import copy

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rospit_msgs.action import ExecuteXMLTestSuite
from rospit_msgs.msg import Condition as ConditionMessage, \
                            ConditionEvaluationPair as CEPMessage, \
                            ConditionEvaluationPairStamped, \
                            Evaluation as EvaluationMessage, \
                            TestCase as TestCaseMessage, \
                            TestCaseReport as TestCaseReportMessage, \
                            TestSuiteReport as TestSuiteReportMessage
from rospit_msgs.srv import MBTIterator

from .ros_parameters import map_msg_to_param


INVARIANT_EVALUATIONS_TOPIC = '/invariant_evaluations'


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


class ROSTestRunnerNode(Node):
    """A node that runs tests and publishes their results."""

    def __init__(self, executor):
        """Initialize."""
        super().__init__('test_runner')
        self.node_executor = executor
        self.invariant_evaluations = []

        self.invariant_evaluation_subscription = self.create_subscription(
            ConditionEvaluationPairStamped, INVARIANT_EVALUATIONS_TOPIC,
            self.add_invariant_evaluation, 10)
        # prevent unused variable warning
        self.invariant_evaluation_subscription
        self.spinning = False
        self._action_server = ActionServer(
                self,
                ExecuteXMLTestSuite,
                'execute_xml_test_suite',
                self.execute_xml_test_suite)
        self.active_goal_handle = None

    def execute_xml_test_suite(self, goal_handle):
        """Execute a test suite specified in an XML file."""
        iterator = goal_handle.request.iterator
        self.active_goal_handle = goal_handle
        self.get_logger().info('Executing test suite')
        result = ExecuteXMLTestSuite.Result()

        from rospit2.rospit_xml import get_test_suite_from_xml_path
        if not goal_handle.request.path:
            result.success = False
            result.error = 'No path to test description specified'
            return result

        parser = get_test_suite_from_xml_path(
            self, goal_handle.request.path, True)
        if not parser:
            result.success = False
            result.error = 'Failed to parse the file specified at path'
            return result

        if iterator:
            iterator_client = self.create_client(MBTIterator, iterator)

        if goal_handle.request.iterations != 0:
            iterate = True
            i = 0
            while iterate:
                parameters = [copy.deepcopy(parameter) for parameter
                              in goal_handle.request.parameters]
                if iterator:
                    request = MBTIterator.Request()
                    request.iteration = i
                    response = iterator_client.call(request)
                    for p1 in response.parameters:
                        for p in parameters:
                            if p.node_name == p1.node_name and \
                               p.parameter_name == p1.parameter_name:
                                p.parameter_value = p1.parameter_value
                                break
                        parameters.append(p1)
                self.execute_test_suite(
                    parser, parameters, result)
                if goal_handle.request.iterations == -1:
                    continue
                i += 1
                iterate = i < goal_handle.request.iterations
            goal_handle.succeed()
        else:
            self.execute_test_suite(
                parser, goal_handle.request.parameters, result)
            goal_handle.succeed()

        return result

    def execute_test_suite(self, parser, parameters, result):
        test_suite = parser.parse()

        if test_suite is None:
            result.success = False
            result.error = 'No test suite loaded, call execute_xml_test_suite'
            return result

        parameter_msgs = [
            map_msg_to_param(param_msg) for param_msg
            in parameters]
        report = test_suite.run(self.get_logger(), parameter_msgs)
        mapped_report = map_test_suite_report(report)
        result.success = True
        result.report = mapped_report

    def add_invariant_evaluation(self, evaluation):
        """Store the invariant evaluation."""
        self.invariant_evaluations.append(evaluation)

    def spin(self):
        """Spin the node."""
        self.spinning = True
        self.get_logger().info('Test runner ready')
        rclpy.spin(self, self.node_executor)

    def report_executing(self, test_suite, test_case):
        """Report to action server which suite and case are being executed."""
        if not self.active_goal_handle:
            self.get_logger().error('not currently executing a goal')
        feedback_msg = ExecuteXMLTestSuite.Feedback()
        feedback_msg.state = 'Executing'
        feedback_msg.active_test_suite_name = test_suite
        feedback_msg.active_test_case_name = test_case
        self.active_goal_handle.publish_feedback(feedback_msg)


def main(args=None):
    """Run the test script."""
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ROSTestRunnerNode(executor)
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()
