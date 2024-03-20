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

#ifndef PERCEPTION__IS_MOVING_HPP_
#define PERCEPTION__IS_MOVING_HPP_

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
#include "tf2_ros/transform_broadcaster.h"

#include "perception_system/PerceptionListener.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace perception
{

using namespace std::chrono_literals;

class IsMoving : public BT::ConditionNode
{
public:
  explicit IsMoving(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("frame"),
        BT::InputPort<float>(
          "velocity_tolerance",
          "Tolerance in velocity to consider the human stopped"),
        BT::InputPort<float>("threshold_time", "Lag time to consider the human stopped"),
        BT::InputPort<float>("position_buffer_dimension", "Human position buffer dimension")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;

  std::string frame_, cam_frame_;
  float velocity_tolerance_, threshold_time_, position_buffer_dimension_;
  bool buffer_dim_riched_ = false;
  bool entity_stopped_ = false;
  rclcpp::Time first_time_stopped_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<geometry_msgs::msg::TransformStamped> position_buffer_;

  void add_position(const geometry_msgs::msg::TransformStamped & new_position);
  double compute_velocity();
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;

};

}  // namespace perception

#endif  // PERCEPTION__IS_MOVING_HPP_
