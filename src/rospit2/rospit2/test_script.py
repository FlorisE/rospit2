# Copyright (c) 2020 AIST.
# National Institute of Advanced Industrial Science and Technology.
#
# License: MIT.

"""Script that launches PIT nodes and services."""


import rclpy
from rclpy.executors import MultiThreadedExecutor

from .ros import ROSTestRunnerNode


def main(args=None):
    """Run the test script."""
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ROSTestRunnerNode(executor)
    executor.add_node(node)
    node.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
