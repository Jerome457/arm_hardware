cmake_minimum_required(VERSION 3.10)
project(motion_sdk)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(motion_sdk
  src/usb_serial.cpp
  src/servo.cpp
  src/stepper.cpp
)

target_include_directories(motion_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS motion_sdk
  EXPORT export_motion_sdk
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_motion_sdk HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
