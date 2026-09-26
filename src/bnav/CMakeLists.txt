cmake_minimum_required(VERSION 3.20)
project(bnav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_ros_common REQUIRED)

install(DIRECTORY
  config
  launch
  maps
  bt
  DESTINATION share/${PROJECT_NAME}
)

add_library(random_pose_in_square_bt_node SHARED src/random_pose_in_square.cpp)
target_include_directories(random_pose_in_square_bt_node PRIVATE include)
target_link_libraries(random_pose_in_square_bt_node
  behaviortree_cpp::behaviortree_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
)
install(TARGETS random_pose_in_square_bt_node
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

add_executable(alternate_goals_mission src/alternate_goals_mission_node.cpp)
target_link_libraries(alternate_goals_mission
  rclcpp::rclcpp
  nav2_behavior_tree::nav2_behavior_tree
  nav2_ros_common::nav2_ros_common
  ament_index_cpp::ament_index_cpp
)
install(TARGETS alternate_goals_mission DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
