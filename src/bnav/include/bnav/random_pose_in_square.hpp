#ifndef BNAV__RANDOM_POSE_IN_SQUARE_HPP_
#define BNAV__RANDOM_POSE_IN_SQUARE_HPP_

#include <cmath>
#include <random>

#include "behaviortree_cpp/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace bnav
{

class RandomPoseInSquare : public BT::SyncActionNode
{
public:
  RandomPoseInSquare(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config), gen_(std::random_device{}()) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("center_x", 0.0, "square center x (m)"),
      BT::InputPort<double>("center_y", 0.0, "square center y (m)"),
      BT::InputPort<double>("half_size", 1.0, "half side length (m) — 1.0 = 2m square"),
      BT::InputPort<std::string>("frame_id", "map", "frame the pose is in"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
    };
  }

  BT::NodeStatus tick() override
  {
    double cx = 0.0, cy = 0.0, half = 1.0;
    std::string frame_id = "map";
    getInput("center_x", cx);
    getInput("center_y", cy);
    getInput("half_size", half);
    getInput("frame_id", frame_id);

    std::uniform_real_distribution<double> pos_dist(-half, half);
    std::uniform_real_distribution<double> yaw_dist(-M_PI, M_PI);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = cx + pos_dist(gen_);
    pose.pose.position.y = cy + pos_dist(gen_);
    double yaw = yaw_dist(gen_);
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);

    setOutput("goal", pose);
    return BT::NodeStatus::SUCCESS;
  }

private:
  std::mt19937 gen_;
};

}  // namespace bnav
#endif