cmake_minimum_required(VERSION 3.16)
project(stepper_foc_1_hw LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(stepper_foc_1_hw SHARED src/stepper_foc_1_hw.cpp)

target_include_directories(stepper_foc_1_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# SocketCAN is in libc (linux/can/raw.h); nothing extra to link.
target_link_libraries(stepper_foc_1_hw
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface stepper_foc_1_hw.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS stepper_foc_1_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY bringup DESTINATION share/${PROJECT_NAME})
install(FILES stepper_foc_1_hw.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(stepper_foc_1_hw)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()