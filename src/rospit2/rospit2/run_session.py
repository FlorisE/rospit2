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

"""Script that launches a ROSPIT session."""

from threading import Thread

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from rospit_msgs.action import ExecuteXMLTestSuite
from rospit_msgs.srv import MBTInitializer

from .ros_parameters import map_param_to_msg
from .rospit_session_xml import get_session_from_xml_path
from .rospit_test_runner import ROSTestRunnerNode


class ExecuteCall(Thread):
    """Thread that executes the call to the action server."""

    def __init__(self, node, action_client):
        """Initialize."""
        super().__init__()
        self.node = node
        self.action_client = action_client

    def run(self):
        """Run."""
        param = self.node.get_parameter('path')
        path = param.get_parameter_value().string_value

        if not path:
            self.node.get_logger().error('Path has not been specified')
            return
        parser = get_session_from_xml_path(self, path)

        if not parser:
            self.node.get_logger().error('Could not initialize parser.')
            return

        session = parser.parse()

        if not session:
            self.node.get_logger().error(
                'Could not parse the session definition.')
            return

        for episode in session.episodes:
            self.node.get_logger().info(
                f'Executing test episode {episode.name}')
            if episode.initialize_service:
                self.node.get_logger().info(
                    f'Initializer: {episode.initialize_service}')
                client = self.node.create_client(MBTInitializer,
                                                 episode.initialize_service)
                request = MBTInitializer.Request()
                client.call(request)
            if episode.iterate_service:
                self.node.get_logger().info(
                    f'Iterator: {episode.iterate_service}')
                if episode.iterations == -1:
                    self.node.get_logger().info(
                        'Running until oracle terminates')
                else:
                    self.node.get_logger().info(
                        f'Running for {episode.iterations} iteration(s)')

            goal_msg = ExecuteXMLTestSuite.Goal()
            goal_msg.path = episode.path
            goal_msg.parameters = [
                map_param_to_msg(parameter) for parameter
                in episode.parameters]
            if episode.initialize_service:
                goal_msg.initializer = episode.initialize_service
            if episode.iterate_service:
                goal_msg.iterator = episode.iterate_service
            if episode.iterations:
                goal_msg.iterations = int(episode.iterations)
            self.action_client.wait_for_server()
            response = self.action_client.send_goal(goal_msg)
            self.node.get_logger().info(
                f'Status: {response.status}')


class ExecuteSpin(Thread):
    """Thread that spins the executor."""

    def __init__(self, node, executor):
        """Initialize."""
        super().__init__()
        self.node = node
        self.executor = executor
        self.spinning = True

    def run(self):
        """Run."""
        while self.spinning:
            self.executor.spin_once()


def main(args=None):
    """Run a ROSPIT test session."""
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ROSTestRunnerNode(executor)
    action_client = ActionClient(
        node,
        ExecuteXMLTestSuite,
        '/execute_xml_test_suite')
    node.declare_parameter('path')
    executor.add_node(node)
    t1 = ExecuteCall(node, action_client)
    t2 = ExecuteSpin(node, executor)
    t1.start()
    t2.start()
    t1.join()
    t2.spinning = False
    t2.join()
    rclpy.shutdown()
