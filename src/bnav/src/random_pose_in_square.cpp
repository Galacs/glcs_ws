#include "bnav/random_pose_in_square.hpp"
#include "behaviortree_cpp/bt_factory.h"

// BT_REGISTER_NODES(factory)
// {
//   factory.registerNodeType<bnav::RandomPoseInSquare>("RandomPoseInSquare");
// }
extern "C" __attribute__((visibility("default")))
void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<bnav::RandomPoseInSquare>("RandomPoseInSquare");
}