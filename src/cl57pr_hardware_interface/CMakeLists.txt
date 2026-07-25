cmake_minimum_required(VERSION 3.16)
project(cl57pr_hardware_interface LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(fastcdr REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED IMPORTED_TARGET libmodbus)

add_library(
  cl57pr_hardware_interface
  SHARED
  src/cl57pr_hardware_interface.cpp
)

target_include_directories(
  cl57pr_hardware_interface
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  cl57pr_hardware_interface
  PkgConfig::MODBUS
)
target_link_libraries(cl57pr_hardware_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${libmodbus_LIBRARIES}
)

pluginlib_export_plugin_description_file(hardware_interface cl57pr_hardware_interface.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS cl57pr_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY bringup
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES cl57pr_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(cl57pr_hardware_interface)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
