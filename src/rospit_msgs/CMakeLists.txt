cmake_minimum_required(VERSION 3.5)
project(rospit_msgs)

find_package(ament_cmake)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/Condition.msg"
  "msg/ConditionEvaluationPair.msg"
  "msg/ConditionEvaluationPairStamped.msg"
  "msg/Evaluation.msg"
  "msg/EvaluationStamped.msg"
  "msg/MovementEvaluation.msg"
  "msg/MovementEvaluationStamped.msg"
  "msg/Point2DEvaluation.msg"
  "msg/Point2DEvaluationStamped.msg"
  "msg/Point2D.msg"
  "msg/PoseEvaluation.msg"
  "msg/PoseEvaluationStamped.msg"
  "msg/TestCase.msg"
  "msg/TestCaseReport.msg"
  "msg/TestSuite.msg"
  "msg/TestSuiteReport.msg")

set(action_files
  "action/ExecuteXMLTestSuite.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs)
ament_package()
