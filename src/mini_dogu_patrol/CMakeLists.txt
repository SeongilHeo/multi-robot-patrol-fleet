cmake_minimum_required(VERSION 3.8)
project(mini_dogu_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(
  patrol_manager
  src/patrol_manager.cpp
)

target_include_directories(
  patrol_manager
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(
  patrol_manager
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(
  patrol_manager
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
)

install(
  TARGETS patrol_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()