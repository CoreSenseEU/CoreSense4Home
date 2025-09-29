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

#include "arm/manipulation/play_motion_predefined.hpp"

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "play_motion2_msgs/action/play_motion2.hpp"

namespace manipulation
{

using namespace std::chrono_literals;
using namespace std::placeholders;

PlayMotionPredefined::PlayMotionPredefined(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: manipulation::BtActionNode<
    play_motion2_msgs::action::PlayMotion2,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(xml_tag_name, action_name, conf)
{
}

void PlayMotionPredefined::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "PlayMotionPredefined ticked");

  // Read ports
  getInput("motion_name", motion_name_);
  getInput("skip_planning", skip_planning_);

  // Fill goal
  goal_.motion_name = motion_name_;
  goal_.skip_planning = skip_planning_;

  RCLCPP_INFO(
    node_->get_logger(), "Requesting PlayMotion2 '%s' (skip_planning=%s)",
    goal_.motion_name.c_str(), skip_planning_ ? "true" : "false");
}

BT::NodeStatus PlayMotionPredefined::on_success()
{
  if (result_.result && result_.result->success) {
    return BT::NodeStatus::SUCCESS;
  } else {
    if (result_.result) {
      RCLCPP_ERROR(node_->get_logger(), "PlayMotion2 failed: %s", result_.result->error.c_str());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "PlayMotion2 failed: no result received");
    }
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace manipulation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      // Default action server name as per play_motion2: "/play_motion2"
      return std::make_unique<manipulation::PlayMotionPredefined>(name, "/play_motion2", config);
    };

  factory.registerBuilder<manipulation::PlayMotionPredefined>("PlayMotionPredefined", builder);
}
