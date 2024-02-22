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

#include "arm/manipulation/pick_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "manipulation_interfaces/action/pick.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace manipulation
{

using namespace std::chrono_literals;
using namespace std::placeholders;

PickObject::PickObject(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: manipulation::BtActionNode<manipulation_interfaces::action::Pick>(xml_tag_name,
    action_name, conf) {}

void PickObject::on_tick()
{

  RCLCPP_DEBUG(node_->get_logger(), "Speak ticked");
  moveit_msgs::msg::CollisionObject::SharedPtr object_;
  getInput("object_to_pick", object_);
  goal_.object_goal = *object_;
}

BT::NodeStatus PickObject::on_success() {return BT::NodeStatus::SUCCESS;}

} // namespace manipulation
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string & name,
      const BT::NodeConfiguration & config) {
      return std::make_unique<manipulation::PickObject>(name, "/pick", config);
    };

  factory.registerBuilder<manipulation::PickObject>("PickObject", builder);
}
