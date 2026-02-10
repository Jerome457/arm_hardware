cmake_minimum_required(VERSION 3.10)
project(stepper_hardware_interface)

# -------------------------------------------------
# Dependencies
# -------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(motion_sdk REQUIRED)

# -------------------------------------------------
# Library
# -------------------------------------------------
add_library(stepper_hardware SHARED
  src/stepper_hardware.cpp
)

target_include_directories(stepper_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(stepper_hardware
  rclcpp
  hardware_interface
  pluginlib
  motion_sdk
)

# -------------------------------------------------
# pluginlib export (CRITICAL)
# -------------------------------------------------
pluginlib_export_plugin_description_file(
  hardware_interface plugin.xml
)

# This line is REQUIRED so the ament index is generated
ament_export_dependencies(hardware_interface)

# -------------------------------------------------
# Install
# -------------------------------------------------
install(
  TARGETS stepper_hardware
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------------------------------
# Package
# -------------------------------------------------
ament_package()
