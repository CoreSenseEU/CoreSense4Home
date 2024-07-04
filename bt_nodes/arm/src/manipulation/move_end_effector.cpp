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

#include "arm/manipulation/move_end_effector.hpp"

#include <math.h>

namespace manipulation
{

using namespace std::chrono_literals;
using namespace std::placeholders;

MoveEndEffector::MoveEndEffector(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: manipulation::BtActionNode<
    manipulation_interfaces::action::MoveEndEffector,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(xml_tag_name, action_name, conf)
{
}

void MoveEndEffector::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "MoveEndEffector ticked");
  getInput("pose", pose_to_move_);

  goal_.pose = pose_to_move_;
    
  RCLCPP_INFO(
    node_->get_logger(), "Moving to %f %f %f", goal_.pose.pose.position.x,
    goal_.pose.pose.position.y, goal_.pose.pose.position.z);
  
  return;
  
}

BT::NodeStatus MoveEndEffector::on_success()
{
  if (result_.result->success) {
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "MoveEndEffector failed");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace manipulation
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<manipulation::MoveEndEffector>(name, "move_end_effector", config);
    };

  factory.registerBuilder<manipulation::MoveEndEffector>("MoveEndEffector", builder);
}
