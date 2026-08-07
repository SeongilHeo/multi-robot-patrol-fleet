cmake_minimum_required(VERSION 3.8)
project(mini_dogu_scheduler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mini_dogu_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(
  mission_scheduler_node
  src/mission_scheduler_node.cpp
)

target_compile_features(
  mission_scheduler_node
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(
  mission_scheduler_node
  rclcpp
  geometry_msgs
  mini_dogu_interfaces
  std_msgs
  std_srvs
)

install(
  TARGETS
    mission_scheduler_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
