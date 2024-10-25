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

#include "motion/head/set_joint_position.hpp"

#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace head
{
SetHeadJointPosition::SetHeadJointPosition(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: motion::BtActionNode<
    control_msgs::action::FollowJointTrajectory,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
  config().blackboard->get("node", node_);
}

void SetHeadJointPosition::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "SetTorsoHeight ticked");

  getInput("vertical", vertical_);
  getInput("horizontal", horizontal_);

  goal_.trajectory.joint_names = {"head_1_joint", "head_2_joint"};
  goal_.trajectory.points.resize(1);
  goal_.trajectory.points[0].positions = {horizontal_, vertical_};
  goal_.trajectory.points[0].time_from_start = rclcpp::Duration(1, 0);
}

BT::NodeStatus SetHeadJointPosition::on_success() {return BT::NodeStatus::SUCCESS;}

}  // namespace manipulation
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<head::SetHeadJointPosition>(
        name,
        "/head_controller/follow_joint_trajectory",
        config);
    };

  factory.registerBuilder<head::SetHeadJointPosition>("SetHeadJointPosition", builder);
}
