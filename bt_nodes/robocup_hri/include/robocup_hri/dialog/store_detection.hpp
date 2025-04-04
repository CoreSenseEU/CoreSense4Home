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

#ifndef robocup_hri__GENERATE_TEXT_FROM_OBJECTS_HPP_
#define robocup_hri__GENERATE_TEXT_FROM_OBJECTS_HPP_

#include <chrono>
#include <functional>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace robocup_hri
{

class StoreDetection : public BT::ActionNodeBase
{
public:
  explicit StoreDetection(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<std::string>("current_name"),
        BT::InputPort<std::string>("drink"),
        BT::InputPort<std::string>("guest_id"),
        BT::InputPort<std::int64_t>("guest_color_id"),
        BT::InputPort<std::string>("guest_description"),

        BT::OutputPort<std::string>("name_1"),
        BT::OutputPort<std::string>("name_2"),
        BT::OutputPort<std::string>("drink_1"),
        BT::OutputPort<std::string>("drink_2"),
        BT::OutputPort<std::int64_t>("guest_color_id_1"),
        BT::OutputPort<std::int64_t>("guest_color_id_2"),
        BT::OutputPort<std::string>("guest_description_id_1"),
        BT::OutputPort<std::string>("guest_description_id_2")});
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;

  std::string current_name_, current_drink_, name_1_, name_2_, drink_1_, drink_2_, current_description_;
  std::string current_id_ = "0";
  std::int64_t current_color_id_ = 0;
};

}  // namespace robocup_hri

#endif  // robocup_hri__GENERATE_TEXT_FROM_OBJECTS_HPP_
