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

#ifndef PERCEPTION__IS_ENTITY_MOVING_HPP_
#define PERCEPTION__IS_ENTITY_MOVING_HPP_

#include <string>
#include <algorithm>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h"

#include "perception_system/PerceptionListener.hpp"

#include "rclcpp/rclcpp.hpp"

namespace perception
{

using namespace std::chrono_literals;

class IsEntityMoving : public BT::ConditionNode
{
public:
  explicit IsEntityMoving(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("frame"),
        BT::InputPort<int>("max_iterations"),
        BT::InputPort<float>(
          "velocity_tolerance",
          "velocity tolerance to consider the entity is moving")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;

  std::string frame_, cam_frame_;
  float velocity_tolerance_;

  std::vector<geometry_msgs::msg::TransformStamped> entity_transforms_;
  std::vector<float> velocities_;
  int max_iterations_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace perception

#endif  // PERCEPTION__IS_ENTITY_MOVING_HPP_
