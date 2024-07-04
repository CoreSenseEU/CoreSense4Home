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

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "perception_system/PerceptionListener.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace perception
{

using namespace std::chrono_literals;

class IsEntityMoving : public BT::ConditionNode
{
public:
  explicit IsEntityMoving(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<std::string>("frame"),
        BT::InputPort<float>("check_time", "time in seconds to check if the entity is moving"),
        BT::InputPort<float>(
          "distance_tolerance", "distance tolerance to consider the entity is moving"),
        BT::InputPort<float>(
          "robot_distance_to_person",
          "distance between the robot and the person")});
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;

  std::string frame_, cam_frame_;
  float distance_tolerance_, check_time_, robot_distance_to_person_;

  std::vector<geometry_msgs::msg::TransformStamped> entity_transforms_;
  std::vector<float> velocities_;
  int max_iterations_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Time time_since_last_stop_;
  rclcpp::Time first_time_;
  bool has_stoped_{false};
};

}  // namespace perception

#endif  // PERCEPTION__IS_ENTITY_MOVING_HPP_
