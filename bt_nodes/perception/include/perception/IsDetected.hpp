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

#ifndef PERCEPTION__ISDETECTED_HPP_
#define PERCEPTION__ISDETECTED_HPP_

#include <string>
#include <algorithm>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "rclcpp/rclcpp.hpp"

namespace perception
{

class IsDetected : public BT::ConditionNode
{
public:
  explicit IsDetected(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({
      BT::InputPort<geometry_msgs::msg::PoseStamped>("max_entities"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("entity"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("confidence"),
      BT::InputPort<std::string>("order"),
      BT::InputPort<double>("max_depth"),
      BT::OutputPort<std::vector<std::string>>("frames")
    });
  }

private:
  rclcpp::Node::SharedPtr node_;

  std::string entity_, order_;
  double threshold_, max_depth_;
  int max_entities_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;  
};

}  // namespace perception

#endif  // PERCEPTION__ISDETECTED_HPP_
