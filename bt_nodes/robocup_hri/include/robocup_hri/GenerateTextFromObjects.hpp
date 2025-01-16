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
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace robocup_hri
{

class GenerateTextFromObjects : public BT::ActionNodeBase
{
public:
  explicit GenerateTextFromObjects(
    const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::vector<moveit_msgs::msg::CollisionObject::SharedPtr>>(
          "detected_objects"),
        BT::OutputPort<std::string>("output_text"),
        BT::OutputPort<moveit_msgs::msg::CollisionObject::SharedPtr>("selected_object"),
      });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  unsigned int selected_object_ = 0;
};

}  // namespace robocup_hri

#endif  // robocup_hri__GENERATE_TEXT_FROM_OBJECTS_HPP_
