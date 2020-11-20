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
`framework.py` contains a low level testing framework, mostly similar to frameworks such as JUnit or xUnit, but with the addition of pre- and post-conditions and invariants.
Conditions can be binary, categorical or numeric, defined in `binary.py`, `category.py` and `numeric.py`, respectively.
`declarative.py` contains an abstraction of the testing framework that allows high level tools to define test execution using `Step` building blocks.
`rospit_xml.py` contains a parser for tests conforming to this schema.
`ros.py` contains an executor for the parsed tests and an implementation of various ROS specific `Steps` and `Conditions`.

### Execution flow

The top level element is a test suite. Execution of a test suite proceeds as follows:

- Execute one time set up steps for test suite.
  - Execute each test case.
- Execute one time tear down steps for test suite.

Test case execution proceeds as follows:

- If `depends_on_previous` has been set for the test case, check if execution of previous test case was nominal.
- Execute (shared) set up steps declared by the test suite.
- Execute set up steps declared by the test case.
- Evaluate preconditions declared by the test case, if any is invalid, block if `wait_for_preconditions` is set, else terminate.
- Execute run steps declared by the test case, while in parallel monitoring invariants (conditions on topic messages).
- Evaluate postconditions declared by the test case.
- Execute tear down steps declared by the test case.
- Execute (shared) tear down steps declared by the test suite.

### How to run the turlesim example
A simple example test script is included. Follow these instructions to run it:
* Make sure ROS 2 Foxy ([installation instructions](https://index.ros.org/doc/ros2/Installation/Foxy/)) and turtlesim ([see this tutorial](https://index.ros.org/doc/ros2/Tutorials/Turtlesim/Introducing-Turtlesim/)) are installed.
* Open a terminal and clone this repository into a new workspace.
* Source your ROS 2 installation, e.g. `source /opt/ros/foxy/setup.bash`.
* Run `colcon build` in the workspace.
* Source the workspace, e.g. `source install/local_setup.bash`.
* Start the test runner service: `python3 src/rospit2/scripts/run_tests.py`
* Open a new terminal, navigate to the workspace, source your ROS 2 installation and the workspace.
* Call the action server for executing XML test scripts: `ros2 action send_goal /execute_xml_test_suite rospit_msgs/action/ExecuteXMLTestSuite "{ path: <PATH TO WORK SPACE>/src/rospit2/examples/move_turtlesim.xml }"`
* Turtlesim should start automatically, the turtle should move, meanwhile the movement will be verified by ROSPIT. Afterwards turtlesim will automatically be closed.

A more complex example using a CRANE X7 robot arm can be found in [this repository](https://github.com/FlorisE/crane_pnp_pits/). The example has not been ported to ROS 2 yet.

## rospit_msgs

This package contains various message definitions as well as the definition of an action server for executing tests.
