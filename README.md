# rospit2
Physical Integration Testing for ROS 2.

## How to run the turlesim example
A simple example test script is included. Follow these instructions to run it:
* Make sure ROS 2 Foxy ([installation instructions](https://index.ros.org/doc/ros2/Installation/Foxy/)) and turtlesim ([see this tutorial](https://index.ros.org/doc/ros2/Tutorials/Turtlesim/Introducing-Turtlesim/)) are installed.
* Open a terminal and clone this repository into a new workspace.
* Source your ROS 2 installation, e.g. `source /opt/ros/foxy/setup.bash`.
* Run `colcon build` in the workspace.
* Start turtlesim: `ros2 run turtlesim turtlesim_node`.
* Open a new terminal, navigate to the workspace and source the workspace: `source install/local_setup.bash`.
* Start the test runner service: `python3 src/rospit2/scripts/run_tests.py`
* Open a new terminal, navigate to the workspace, source your ROS 2 installation (`source /opt/ros/foxy/setup.bash`) and the workspace: `source install/local_setup.bash`.
* Call the action server for executing XML test scripts: `ros2 action send_goal /execute_xml_test_suite rospit_msgs/action/ExecuteXMLTestSuite "{ path: <PATH TO WORK SPACE>/src/rospit2/examples/move_turtlesim.xml }"`
* The turtle should move and reset after moving, meanwhile the movement will be verified by ROSPIT.

A more complex example using a CRANE X7 robot arm can be found in [this repository](https://github.com/FlorisE/crane_pnp_pits/).
