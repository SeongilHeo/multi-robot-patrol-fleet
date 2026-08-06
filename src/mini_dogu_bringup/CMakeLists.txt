cmake_minimum_required(VERSION 3.8)
project(mini_dogu_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(
  system_readiness_supervisor
  src/system_readiness_supervisor.cpp
)

target_compile_features(
  system_readiness_supervisor
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(
  system_readiness_supervisor
  nav2_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
  lifecycle_msgs
)

install(
  TARGETS
    system_readiness_supervisor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()