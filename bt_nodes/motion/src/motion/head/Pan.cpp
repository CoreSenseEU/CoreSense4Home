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

#include "motion/head/Pan.hpp"

namespace head
{

using namespace std::chrono_literals;

Pan::Pan(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), phase_(0.0)
{
  config().blackboard->get("node", node_);
  // joint_range_ = 20.0 * M_PI / 180.0;
  getInput("range", joint_range_);
  joint_range_ = joint_range_ * M_PI / 180.0;
  getInput("period", period_);
  getInput("pitch_angle", pitch_angle_);
  pitch_angle_ = pitch_angle_ * M_PI / 180.0;

  // if (!joint_range_) {
  //   // throw BT::RuntimeError("Missing required input [range]: ", joint_range_);
  //   RCLCPP_WARN(
  //     node_->get_logger(), "Missing required input [range]. Using default value 45.0 degrees");
  //   joint_range_.value() = 45.0 * M_PI / 180.0;
  // }
  // if (!period_) {
  //   // throw BT::RuntimeError("Missing required input [period]: ", period_);
  //   RCLCPP_WARN(
  //     node_->get_logger(), "Missing required input [period]. Using default value 5.0 seconds");
  //   period_.value() = 5.0;
  // }
  // if (!pitch_angle_) {
  //   // throw BT::RuntimeError("Missing required input [pitch_angle]: ", pitch_angle_);
  //   RCLCPP_WARN(
  //     node_->get_logger(), "Missing required input [pitch_angle]. Using default value 0.0 degrees");
  //   pitch_angle_.value() = 0.0;
  // }
  RCLCPP_DEBUG(
    node_->get_logger(), "Pan: range: %f, period: %f, pitch_angle: %f", joint_range_, period_,
    pitch_angle_);
  attention_cmd_pub_ = node_->create_publisher<attention_system_msgs::msg::AttentionCommand>(
    "attention/attention_command", 1);
  attention_cmd_pub_->on_activate();

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 100, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "head_1_joint") {  // TODO: remove hardcoded joint name (TIAGo specific)
          phase_ = msg->position[i];
        } else if (msg->name[i] == "head_2_joint") {
          pitch_ = msg->position[i];
        }
      }
    });
}

void Pan::halt()
{
  // node_->add_activation("attention_server");
}

double Pan::get_joint_yaw(double period, double range, double time, double phase)
{
  return std::clamp(
    range * sin((2 * M_PI / period) * time + phase), -1.3,
    1.3);  // TODO: remove hardcoded limits (TIAGo specific)
}

BT::NodeStatus Pan::tick()
{
  rclcpp::spin_some(node_->get_node_base_interface());
  // bool is_first_tick = false;

  if (status() == BT::NodeStatus::IDLE) {
    config().blackboard->get("tf_broadcaster", tf_broadcaster_);
    // node_->remove_activation("attention_server");
    start_time_ = node_->now();
    initial_yaw_ = phase_;  // Store the actual starting position
    // is_first_tick = true;

    // Calculate phase so the sine wave starts exactly at the current position
    // The sine wave equation is: yaw = range * sin(2π/period * t + phase)
    // At t=0, we want: initial_yaw = range * sin(phase)
    // Therefore: phase = asin(initial_yaw / range)

    // Clamp the ratio to valid range for asin [-1, 1]
    double ratio = initial_yaw_ / joint_range_;

    // If we're outside the range, clamp to the range limit
    if (std::abs(ratio) > 1.0) {
      ratio = (ratio > 0) ? 1.0 : -1.0;
    }

    phase_offset_ = asin(ratio);

    RCLCPP_INFO(
      node_->get_logger(),
      "Pan initialized: initial_yaw=%f rad, range=%f rad, calculated phase_offset=%f rad",
      initial_yaw_, joint_range_, phase_offset_);

    attention_system_msgs::msg::AttentionCommand attention_command_msg;
    attention_command_msg.frame_id_to_track = "pan_target";
    attention_cmd_pub_->publish(attention_command_msg);
  }

  auto elapsed = node_->now() - start_time_;

  double yaw = get_joint_yaw(period_, joint_range_, elapsed.seconds(), phase_offset_);
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 5000,
    "Pan: current_yaw: %f, desired_yaw: %f, elapsed: %.2f", phase_, yaw, elapsed.seconds());

  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = node_->now();
  transform_msg.header.frame_id = "torso_lift_link";
  transform_msg.child_frame_id = "pan_target";

  double r = 5.0;
  transform_msg.transform.translation.x = r * cos(yaw) * cos(pitch_angle_) + 0.182;
  transform_msg.transform.translation.y = r * sin(yaw) * cos(pitch_angle_);
  transform_msg.transform.translation.z = r * sin(pitch_angle_);

  transform_msg.transform.rotation.x = 0.0;
  transform_msg.transform.rotation.y = 0.0;
  transform_msg.transform.rotation.z = 0.0;
  transform_msg.transform.rotation.w = 1.0;

  if (tf_broadcaster_) {
    tf_broadcaster_->sendTransform(transform_msg);
  } else {
    RCLCPP_ERROR_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Pan: tf_broadcaster_ is null!");
  }

  rclcpp::spin_some(node_->get_node_base_interface());

  return BT::NodeStatus::RUNNING;
}

}  // namespace head

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<head::Pan>("Pan");
}
