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
* A ROS Test Runner node (started using `ros2 run rospit2 test_runner`).
* A ROSPIT Test Session Execution script (started using `ros2 run rospit2 session --ros-args -p path:=path_to_test_session.xml`).
* A ROSPIT Test Suite Execution script (started using `ros2 run rospit2 suite --ros-args -p path:=path_to_test_suite.xml`).

A major component of ROSPIT2 is the XML specification for test suites, for which a schema can be found [here](src/rospit2/rospit2/xml/rospit.xsd).
Additionally there is an XML specification for parameterized execution of tests as sessions and episodes, for which a schema can be found [here](src/rospit2/rospit2/xml/session.xsd).
ROSPIT2 contains the following modules:
* `framework.py` contains a low level testing framework, mostly similar to frameworks such as JUnit or xUnit, but with the addition of pre- and post-conditions and invariants.
* Conditions can be binary, categorical or numeric, defined in `binary.py`, `category.py` and `numeric.py`, respectively.
* `declarative.py` contains an abstraction of the testing framework that allows high level tools to define test execution using `Step` building blocks.
* `rospit_session_xml.py` contains a parser for parameterized test executions conforming to the `sessions.xsd` schema.
* `rospit_xml.py` contains a parser for tests conforming to the `rospit.xsd` schema.
* `rospit_test_runner.py` contains a ROS node for running tests, which runs continuously.
* `ros_session.py` contains a script for launching a test runner, executing a test and then automatically terminating.
* `ros.py` contains an executor for the parsed tests and an implementation of various ROS specific `Conditions`.
* `ros_steps.py` contains implementations of various ROS specific `Steps`.
* `ros_parameters.py` contains support for getting/setting ROS parameters in tests.

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

It is also possible to automatically run multiple test suites in sequence, with each episode getting assigned different test parameters. 
See the `turtlesim_session.xml` example for how to accomplish this.

### How to run the turlesim example
A simple example test script is included. Follow these instructions to run it:
* Make sure ROS 2 Foxy ([installation instructions](https://index.ros.org/doc/ros2/Installation/Foxy/)) and turtlesim ([see this tutorial](https://index.ros.org/doc/ros2/Tutorials/Turtlesim/Introducing-Turtlesim/)) are installed.
* Open a terminal and clone this repository into a new workspace.
* Source your ROS 2 installation, e.g. `source /opt/ros/foxy/setup.bash`.
* Run `colcon build` in the workspace.
* Source the workspace, e.g. `source install/local_setup.bash`.
* Run the turtlesim example using `ros2 run rospit2 suite --ros-args -p path:=src/rospit2/examples/move_turtlesim_embedded_parameters.xml`.
* Turtlesim should start automatically, the turtle should move, meanwhile the movement will be verified by ROSPIT. Afterwards turtlesim will automatically be closed.

You can find the following variants of move_turtlesim:
* `move_turtlesim_no_invariants_no_parameters.xml`: Moves the turtlesim using position and velocity hardcoded in the test suite, does not check for invariants.
* `move_turtlesim_no_parameters.xml`: Moves the turtlesim using position and velocity hardcoded in the test suite, while checking for invariants.
* `move_turtlesim_embedded_parameters.xml`: Moves the turtlesim using position and velocity specified as parameters within the test suite, while checking for invariants.
* `move_turtlesim_requiring_parameters.xml`: Moves the turtlesim according to parameters that should be supplied to the test suite by using sessions and episodes.
* `move_turtlesim_using_parameters_file.xml`: Moves the turtlesim according to parameters specified in a parameters file (in this case using `move_turtlesim_parameters.xml`).
* `move_turtlesim_session.xml`: Moves the turtlesim using a session, relies on the `move_turtlesim_requiring_parameters.xml` test suite.

A more complex example using a CRANE X7 robot arm can be found in [this repository](https://github.com/FlorisE/crane_pnp_pits/). The example has not been ported to ROS 2 yet.

## rospit_msgs

This package contains various message definitions as well as the definition of an action server for executing tests.
