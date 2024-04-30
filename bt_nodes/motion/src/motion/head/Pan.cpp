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

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"


namespace head
{

using namespace std::chrono_literals;

Pan::Pan(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  joint_range_ = 20.0 * M_PI / 180.0;
  joint_range_ = getInput<double>("range").value() * M_PI / 180.0;
  period_ = getInput<double>("period").value();
  pitch_angle_ = getInput<double>("pitch_angle").value() * M_PI / 180.0;

  if (!joint_range_) {
    // throw BT::RuntimeError("Missing required input [range]: ", joint_range_);
    RCLCPP_WARN(
      node_->get_logger(), "Missing required input [range]. Using default value 45.0 degrees");
    joint_range_.value() = 45.0 * M_PI / 180.0;
  }
  if (!period_) {
    // throw BT::RuntimeError("Missing required input [period]: ", period_);
    RCLCPP_WARN(
      node_->get_logger(), "Missing required input [period]. Using default value 5.0 seconds");
    period_.value() = 5.0;
  }
  if (!pitch_angle_) {
    // throw BT::RuntimeError("Missing required input [pitch_angle]: ", pitch_angle_);
    RCLCPP_WARN(
      node_->get_logger(), "Missing required input [pitch_angle]. Using default value 0.0 degrees");
    pitch_angle_.value() = 0.0;
  }
  joint_cmd_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/head_controller/joint_trajectory", 100);
}

void
Pan::halt()
{
  auto client = node_->create_client<lifecycle_msgs::srv::ChangeState>("/attention_server/change_state");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(node_->get_logger(), "[Pan] waiting for service '/attention_server/change_state' to appear...");
  }
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
  auto result = client->async_send_request(request);
  rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result);
}

double
Pan::get_joint_yaw(double period, double range, double time)
{
  return range * sin((2 * M_PI / period) * time);
}

BT::NodeStatus
Pan::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[Pan] ticked");
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
    auto client = node_->create_client<lifecycle_msgs::srv::ChangeState>("/attention_server/change_state");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(node_->get_logger(), "[Pan] waiting for service '/attention_server/change_state' to appear...");
    }
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    auto result = client->async_send_request(request);
    rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result);
    return BT::NodeStatus::RUNNING;
  }

  trajectory_msgs::msg::JointTrajectory command_msg;
  auto elapsed = node_->now() - start_time_;

  double yaw = get_joint_yaw(period_, joint_range_, elapsed.seconds());

  command_msg.joint_names = std::vector<std::string>{"head_1_joint", "head_2_joint"};
  command_msg.points.resize(1);
  command_msg.points[0].positions.resize(2);
  command_msg.points[0].velocities.resize(2);
  command_msg.points[0].accelerations.resize(2);
  command_msg.points[0].positions[0] = yaw;
  command_msg.points[0].positions[1] = pitch_angle_.value();
  command_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.00);
  joint_cmd_pub_->publish(command_msg);

  return BT::NodeStatus::RUNNING;
}

}  // namespace motion

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<head::Pan>("Pan");
}
