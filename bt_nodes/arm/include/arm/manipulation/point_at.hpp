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

#ifndef ARM_MANIPULATION__POINT_AT_HPP_
#define ARM_MANIPULATION__POINT_AT_HPP_

#include <tf2_ros/buffer.h>

#include <algorithm>
#include <string>

#include "arm/manipulation/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "manipulation_interfaces/action/move_end_effector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace manipulation
{

class PointAt : public manipulation::BtActionNode<
    manipulation_interfaces::action::MoveEndEffector,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit PointAt(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<geometry_msgs::msg::PoseStamped::SharedPtr>("pose_to_point"),
        BT::InputPort<std::string>("tf_frame"), BT::InputPort<std::string>("base_frame"),
        BT::InputPort<double>("low_z"), BT::InputPort<double>("high_z")});
  }

private:
  geometry_msgs::msg::PoseStamped::SharedPtr pose_to_point_;
  std::string tf_frame_, base_frame_;
  tf2_ros::Buffer::SharedPtr tf_buffer_;
  geometry_msgs::msg::TransformStamped transform_;
  double low_z_{0.5}, high_z_{1.1};
};

}  // namespace manipulation

#endif  // ARM_MANIPULATION__POINT_AT_HPP_
