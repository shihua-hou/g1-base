cmake_minimum_required(VERSION 3.8)
project(g1_base_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToTarget.action"
  "srv/ExecuteArmAction.srv"
  "srv/ExecuteCustomAction.srv"
  "srv/MoveRobot.srv"
  "srv/PlayNamedAction.srv"
  "srv/RotateRobot.srv"
  "srv/RunMovementScript.srv"
  "srv/Relocalize.srv"
  "srv/SquatRobot.srv"
  "srv/StopRobot.srv"
  "srv/SetFsmId.srv"
  "srv/Speak.srv"
  "srv/SetVolume.srv"
  "srv/GetFsmId.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
