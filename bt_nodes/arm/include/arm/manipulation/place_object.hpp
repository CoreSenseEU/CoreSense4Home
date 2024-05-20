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

#ifndef ARM_MANIPULATION__PLACE_OBJECT_HPP_
#define ARM_MANIPULATION__PLACE_OBJECT_HPP_

#include <algorithm>
#include <string>

#include "arm/manipulation/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "manipulation_interfaces/action/place.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/buffer.h>
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"

namespace manipulation
{

class PlaceObject : public manipulation::BtActionNode<manipulation_interfaces::action::Place>
{
public:
  explicit PlaceObject(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<moveit_msgs::msg::CollisionObject::SharedPtr>("object_to_place"),
       BT::InputPort<geometry_msgs::msg::PoseStamped>("place_pose"),
       BT::InputPort<std::string>("base_frame")});
  }

private:


  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string tf_to_place_, base_frame_;
  moveit_msgs::msg::CollisionObject::SharedPtr object_;
  geometry_msgs::msg::PoseStamped place_pose_;
  geometry_msgs::msg::TransformStamped transform_to_place_;
  //   std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  //  rclcpp::ActionClient<audio_common_msgs::action::TTS>::SharedPtr
  //  tts_action_;

  // std::string text_;
};

}  // namespace manipulation

#endif  // arm_MANIPULATION__PLACE_OBJECT_HPP_
