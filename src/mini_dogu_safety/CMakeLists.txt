cmake_minimum_required(VERSION 3.8)
project(mini_dogu_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(
  safety_gate_node
  src/safety_gate_node.cpp
)

target_compile_features(
  safety_gate_node
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(
  safety_gate_node
  rclcpp
  geometry_msgs
  std_msgs
)

install(
  TARGETS
    safety_gate_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
