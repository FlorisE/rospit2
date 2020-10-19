"""
Script that launches useful nodes and services for executing Physical Integration Tests
"""
import rclpy
from rospit2.ros import ROSTestRunnerNode
from rclpy.executors import MultiThreadedExecutor


def main(args=None):
  rclpy.init(args=args)
  executor = MultiThreadedExecutor()
  node = ROSTestRunnerNode(executor)
  executor.add_node(node)
  node.spin()

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
