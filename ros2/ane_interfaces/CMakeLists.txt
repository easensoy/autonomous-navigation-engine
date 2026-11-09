cmake_minimum_required(VERSION 3.16)
project(ane_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavigatorStatus.msg"
  "msg/NavigatorEvent.msg"
  "msg/BlockageReport.msg"
  "srv/StartMission.srv"
  "srv/InjectFault.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
