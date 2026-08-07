cmake_minimum_required(VERSION 3.8)
project(mini_dogu_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotHeartbeat.msg"
  "msg/FleetRobotState.msg"
  "msg/FleetState.msg"
  "msg/MissionQueueState.msg"
  "msg/RobotIncident.msg"

  "srv/AssignMission.srv"
  "srv/QueueMission.srv"
  "srv/CancelMission.srv"
  "srv/StartMission.srv"
  "srv/ReportIncident.srv"

  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
