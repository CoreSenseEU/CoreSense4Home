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

#include "motion/head/LookAt.hpp"

namespace head
{

LookAt::LookAt(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.transient_local().reliable();
  attention_points_pub_ = node_->create_publisher<attention_system_msgs::msg::AttentionCommand>(
    "attention/attention_command", 1);
}

BT::NodeStatus LookAt::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "LookAt ticked");
  getInput("tf_frames", tf_frames_);
  getInput("tf_frame", tf_frame_);

  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_DEBUG(node_->get_logger(), "IsPointing ticked");
    config().blackboard->get("tf_buffer", tf_buffer_);
  }

  std::string goal_frame;

  if (tf_frames_.size() == 0 && !tf_frame_.empty()) {
    goal_frame = tf_frame_;
  } else if (tf_frames_.size() > 0) {
    goal_frame = tf_frames_[0];
  } else {
    RCLCPP_ERROR(node_->get_logger(), "No goal frame provided");
    return BT::NodeStatus::RUNNING;
  }

  attention_system_msgs::msg::AttentionCommand attention_command_msg;
  RCLCPP_INFO(node_->get_logger(), "LookAt tf_frame_: %s", goal_frame.c_str());

  attention_command_msg.frame_id_to_track = goal_frame;

  attention_points_pub_->publish(attention_command_msg);

  rclcpp::spin_some(node_);
  RCLCPP_INFO(node_->get_logger(), "LookAt published attention points");
  return BT::NodeStatus::SUCCESS;
}

void LookAt::halt() {RCLCPP_INFO(node_->get_logger(), "LookAt halted");}

}  // namespace navigation

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<head::LookAt>("LookAt");
}