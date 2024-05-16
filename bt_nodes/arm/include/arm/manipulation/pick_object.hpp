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
// See the License for the specific language governing permissions andGO2OBJECT
// limitations under the License.

#ifndef ARM_MANIPULATION__PICK_OBJECT_HPP_
#define ARM_MANIPULATION__PICK_OBJECT_HPP_

#include <algorithm>
#include <string>

#include "manipulation_interfaces/action/pick.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "arm/manipulation/BTActionNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace manipulation
{

class PickObject : public manipulation::BtActionNode<
    manipulation_interfaces::action::Pick, rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit PickObject(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<moveit_msgs::msg::CollisionObject::SharedPtr>(
          "object_to_pick")});
  }
};

} // namespace manipulation

#endif // arm_MANIPULATION__PICK_OBJECT_HPP_
