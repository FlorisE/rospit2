# rospit2
Physical Integration Testing for ROS 2.

## How to run the turlesim example
A simple example test script is included. Follow these instructions to run it:
1. Follow the turtlesim installation instructions from [this tutorial](https://index.ros.org/doc/ros2/Tutorials/Turtlesim/Introducing-Turtlesim/)
2. Start turtlesim `ros2 run turtlesim turtlesim_node`
3. Clone this repository into a workspace and run `colcon build`
4. Start the test runner service: `python3 src/rospit2/src/rospit2/run_tests.py`
5. Call the action server for executing XML test scripts: `ros2 action send_goal /execute_xml_test_suite rospit_msgs/action/ExecuteXMLTestSuite "{ path: <PATH TO WORK SPACE>/src/rospit2/examples/move_turtlebot.xml }"`
6. The turtle should move and reset after moving, meanwhile the movement will be verified by ROSPIT.

A more complex example using a CRANE X7 robot arm can be found in [this repository](https://github.com/FlorisE/crane_pnp_pits/).
