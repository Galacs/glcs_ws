#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2::LifecycleNode>("alternate_goals_mission");
  node->autostart();

  std::vector<std::string> plugin_libs = {
    "nav2_navigate_to_pose_action_bt_node",
    "random_pose_in_square_bt_node",
  };
  auto engine = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_libs, node);

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set<std::chrono::milliseconds>("server_timeout", 20s);
  blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
  blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout", 1000ms);

  auto bt_xml_path = (ament_index_cpp::get_package_share_path("bnav") / "bt" / "alternate.xml").string();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  auto on_loop = [&]() { executor.spin_some(); };

  while (rclcpp::ok()) {
    try {
      auto tree = engine->createTreeFromFile(bt_xml_path, blackboard);
      engine->resetGrootMonitor();
      engine->addGrootMonitoring(&tree, 3003);
      engine->run(&tree, on_loop, [&]() { return !rclcpp::ok(); });
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        node->get_logger(), "Behavior tree failed (%s), retrying in 2s", e.what());
      rclcpp::sleep_for(2s);
    }
  }

  rclcpp::shutdown();
  return 0;
}