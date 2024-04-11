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

#include <string>
#include <utility>

#include "arm/manipulation/move_to_predefined.hpp"
#include "manipulation_interfaces/action/move_to_predefined.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace manipulation
{

using namespace std::chrono_literals;
using namespace std::placeholders;

MoveToPredefined::MoveToPredefined(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: manipulation::BtActionNode<manipulation_interfaces::action::MoveToPredefined>(xml_tag_name,
    action_name, conf) {}

void MoveToPredefined::on_tick()
{

  RCLCPP_DEBUG(node_->get_logger(), "MoveToPredefined ticked");

  getInput("pose", pose_);
  goal_.group_name = "arm_torso";
  goal_.goal_pose = pose_;
}

BT::NodeStatus MoveToPredefined::on_success() {return BT::NodeStatus::SUCCESS;}

} // namespace manipulation
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string & name,
      const BT::NodeConfiguration & config) {
      return std::make_unique<manipulation::MoveToPredefined>(name, "/move_to_predefined", config);
    };

  factory.registerBuilder<manipulation::MoveToPredefined>("MoveToPredefined", builder);
}
