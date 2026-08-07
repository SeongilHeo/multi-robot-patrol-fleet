cmake_minimum_required(VERSION 3.8)
project(mini_dogu_fleet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mini_dogu_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(SQLite3 REQUIRED)

add_executable(
  fleet_manager_node
  src/fleet_manager_node.cpp
  src/mission_store.cpp
)

add_executable(
  robot_heartbeat_node
  src/robot_heartbeat_node.cpp
)

target_include_directories(
  fleet_manager_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(
  fleet_manager_node
  PUBLIC
    cxx_std_17
)

ament_target_dependencies(
  robot_heartbeat_node
  rclcpp
  mini_dogu_interfaces
  std_msgs
)

ament_target_dependencies(
  fleet_manager_node
  rclcpp
  rclcpp_action
  nav2_msgs
  mini_dogu_interfaces
  std_srvs
  std_msgs
  geometry_msgs
)

target_link_libraries(
  fleet_manager_node
  SQLite::SQLite3
)

install(
  TARGETS
    fleet_manager_node
    robot_heartbeat_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
