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

#include <cmath>

namespace head
{

using namespace std::chrono_literals;

Pan::Pan(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: head::BtActionNode<
    control_msgs::action::FollowJointTrajectory,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
      xml_tag_name, action_name, conf)
{
}

void Pan::on_tick()
{
  // Leer parámetros
  getInput("range", joint_range_);
  getInput("period", period_);
  getInput("pitch_angle", pitch_angle_);

  // Convertir a radianes
  //joint_range_ = joint_range_ * M_PI / 180.0;
  pitch_angle_ = pitch_angle_ * M_PI / 180.0;

  // pitch_angle_ = std::clamp(pitch_angle_, -pitch_limit_, pitch_limit_);

  if (current_step_ >= yaw_steps_.size()) {
    RCLCPP_WARN(node_->get_logger(), "Pan: no more scan positions");
    current_step_ = 0;
  }

  const double yaw = yaw_steps_[current_step_];
  // Construir goal
  goal_ = control_msgs::action::FollowJointTrajectory::Goal();
  goal_.trajectory.header.stamp = node_->now(); 
  goal_.trajectory.joint_names = {"head_1_joint", "head_2_joint"};

  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions = { yaw, pitch_angle_ };
  p.time_from_start = rclcpp::Duration::from_seconds(1.5);

  goal_.trajectory.points.clear();
  goal_.trajectory.points.push_back(p);

  RCLCPP_INFO(
    node_->get_logger(),
    "Pan step %ld → yaw=%.2f rad",
    current_step_, yaw
  );

  //build_trajectory(goal_.trajectory.points);

  RCLCPP_DEBUG(node_->get_logger(), "Pan action goal sent");
  current_step_++;

}

BT::NodeStatus Pan::on_success()
{
  RCLCPP_DEBUG(node_->get_logger(), "Pan action finished");
  
  return BT::NodeStatus::SUCCESS;
}

void Pan::build_trajectory(
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> & points)
{
  points.clear();

  const double yaw_left  = std::clamp(-joint_range_, -yaw_limit_, yaw_limit_);
  const double yaw_right = std::clamp( joint_range_, -yaw_limit_, yaw_limit_);

  trajectory_msgs::msg::JointTrajectoryPoint p1;
  p1.positions = {yaw_left, pitch_angle_};
  p1.time_from_start = rclcpp::Duration::from_seconds(period_ * 0.25);

  trajectory_msgs::msg::JointTrajectoryPoint p2;
  p2.positions = {yaw_right, pitch_angle_};
  p2.time_from_start = rclcpp::Duration::from_seconds(period_ * 0.75);

  trajectory_msgs::msg::JointTrajectoryPoint p3;
  p3.positions = {yaw_left, pitch_angle_};
  p3.time_from_start = rclcpp::Duration::from_seconds(period_);

  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
}

}  // namespace head

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<head::Pan>(name, "/head_controller/follow_joint_trajectory", config);
    };

  factory.registerBuilder<head::Pan>("Pan", builder);
}