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
#include <iostream>

#include "motion/navigation/move_along_axis.hpp"


#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace navigation
{

using namespace std::chrono_literals;

MoveAlongAxis::MoveAlongAxis(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  getInput("cmd_vel_topic", cmd_vel_topic_);

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 100);
}

void
MoveAlongAxis::halt()
{
  std::cout << "MoveAlongAxis halt" << std::endl;
}

BT::NodeStatus
MoveAlongAxis::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    getInput("speed", speed_);
    getInput("distance", distance_);
    getInput("axis", axis_);
    start_time_ = node_->now();
  }

  geometry_msgs::msg::Twist vel_msgs;
  if (axis_ == "x") {
    vel_msgs.linear.x = speed_;
  } else if (axis_ == "y") {
    vel_msgs.linear.y = speed_;
  } else if (axis_ == "z") {
    vel_msgs.angular.z = speed_;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Invalid axis: %s", axis_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  vel_pub_->publish(vel_msgs);

  auto elapsed = node_->now() - start_time_;

  if (elapsed < std::abs(distance_) / std::abs(speed_) * 1s) {
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigation::MoveAlongAxis>("MoveAlongAxis");
}
