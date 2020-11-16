# Physical Integration Testing for ROS 2

ROSPIT2 is a testing framework for ROS 2, aimed at testing the integration of the hardware and software of robotic systems (Physical Integration Testing).

This repository contains two ROS packages, found in the `src` folder:
* rospit2: Framework for declaring and executing Physical Integration Tests for ROS 2.
* rospit_msgs: Message specifications for ROS 2.

ROSPIT2 is being developed by the [Software Platform Research Team](https://unit.aist.go.jp/icps/icps-sp/en/about-spr-teams/) at the [Industrial CPS Research Center](https://unit.aist.go.jp/icps/index_en.html), [AIST](https://www.aist.go.jp/index_en.html), Japan.

If you use this work in a research project, please cite:

`Erich, F., Saksena, A., Biggs, G., & Ando, N. (2019, January). Design and development of a physical integration testing framework for robotic manipulators. In 2019 IEEE/SICE International Symposium on System Integration (SII) (pp. 602-607). IEEE.`

## rospit2

The rospit2 package contains:
* The rospit2 framework, written in Python.
* A turtlesim example written using the XML front-end for ROSPIT.
* The `run_tests.py` script that starts a test runner.

A major component of ROSPIT2 is the XML specification for test suites, for which a schema can be found [here](src/rospit2/rospit2/xml/rospit.xsd).

### How to run the turlesim example
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

A more complex example using a CRANE X7 robot arm can be found in [this repository](https://github.com/FlorisE/crane_pnp_pits/). The example has not been ported to ROS 2 yet.

## rospit_msgs

This package contains various message definitions as well as the definition of an action server for executing tests.
