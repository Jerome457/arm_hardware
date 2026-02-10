cmake_minimum_required(VERSION 3.8)
project(arm_bldc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-D_GLIBCXX_USE_CXX11_ABI=1)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(arm_bldc_controller SHARED
  src/arm_bldc_system.cpp
)

target_include_directories(arm_bldc_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/nmotion_transport/include>
  $<INSTALL_INTERFACE:include>
)

add_library(nmotion_transport SHARED IMPORTED)
set_target_properties(nmotion_transport PROPERTIES
  IMPORTED_LOCATION
    ${CMAKE_CURRENT_SOURCE_DIR}/nmotion_transport/lib/libnmotion_transport.so
)

target_link_libraries(arm_bldc_controller
  nmotion_transport
)

ament_target_dependencies(arm_bldc_controller
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface
  plugin.xml
)

install(
  TARGETS arm_bldc_controller
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES nmotion_transport/lib/libnmotion_transport.so
  DESTINATION lib
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
