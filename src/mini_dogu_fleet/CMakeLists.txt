cmake_minimum_required(VERSION 3.8)
project(mini_dogu_fleet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mini_dogu_interfaces REQUIRED)

add_executable(
  fleet_manager_node
  src/fleet_manager_node.cpp
)

ament_target_dependencies(
  fleet_manager_node
  rclcpp
  mini_dogu_interfaces
)

add_executable(
  robot_heartbeat_node
  src/robot_heartbeat_node.cpp
)

ament_target_dependencies(
  robot_heartbeat_node
  rclcpp
  mini_dogu_interfaces
)

install(
  TARGETS
    fleet_manager_node
    robot_heartbeat_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()