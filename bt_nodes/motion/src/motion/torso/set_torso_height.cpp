// Copyright 2024 Intelligent Robotics Lab - Gentlebots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "motion/torso/set_torso_height.hpp"

#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace torso
{
SetTorsoHeight::SetTorsoHeight(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: motion::BtActionNode<
    manipulation_interfaces::action::MoveJoint,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
  config().blackboard->get("node", node_);
}

void SetTorsoHeight::on_tick()
{
  RCLCPP_INFO(node_->get_logger(), "SetTorsoHeight ticked");

  getInput("height", height_);

  goal_.group_name = "arm_torso";
  goal_.joint_name = "torso_lift_joint";
  goal_.joint_value = height_;
}

BT::NodeStatus SetTorsoHeight::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "SetTorsoHeight success");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace manipulation
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<torso::SetTorsoHeight>(name, "/move_joint", config);
    };

  factory.registerBuilder<torso::SetTorsoHeight>("SetTorsoHeight", builder);
}
