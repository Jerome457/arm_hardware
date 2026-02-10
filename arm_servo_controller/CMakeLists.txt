cmake_minimum_required(VERSION 3.10)
project(arm_servo_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(motion_sdk REQUIRED)

add_library(arm_servo_controller SHARED
  src/arm_servo_hardware.cpp
)

target_include_directories(arm_servo_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arm_servo_controller
  rclcpp
  hardware_interface
  pluginlib
  motion_sdk
)

pluginlib_export_plugin_description_file(
  hardware_interface plugin.xml
)

install(
  TARGETS arm_servo_controller
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
