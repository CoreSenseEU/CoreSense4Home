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

#include "motion/base/Rotate.hpp"

namespace base
{

Rotate::Rotate(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    cmd_vel_pub_->on_activate();
}

BT::NodeStatus Rotate::tick()
{
  RCLCPP_INFO(node_->get_logger(), "Rotate ticked");

    getInput("angle", angle_);
    getInput("speed", speed_);

  if (status() == BT::NodeStatus::IDLE) {
    current_angle_ = 0;
    last_time_ = std::chrono::high_resolution_clock::now();
    return BT::NodeStatus::RUNNING;
  }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.angular.z = speed_;

    if (current_angle_ < angle_) {
      cmd_vel_pub_->publish(cmd_vel);
      auto curr_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_time_);
      current_angle_ += speed_ * elapsed.count() / 1000.0;
        last_time_ = curr_time;
      return BT::NodeStatus::RUNNING;
    }

  return BT::NodeStatus::FAILURE;
}

void Rotate::halt()
{
  RCLCPP_INFO(node_->get_logger(), "Rotate halted");
}

}  // namespace navigation

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<base::Rotate>("Rotate");
}
