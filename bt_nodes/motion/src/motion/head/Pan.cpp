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

Pan::Pan(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  phase_(0.0)
{
  config().blackboard->get("node", node_);
  joint_range_ = getInput<double>("range").value() * M_PI / 180.0;
  period_ = getInput<double>("period").value();

  if (!joint_range_) {
    // throw BT::RuntimeError("Missing required input [range]: ", joint_range_);
    joint_range_.value() = 45.0 * M_PI / 180.0;
  }
  if (!period_) {
    // throw BT::RuntimeError("Missing required input [period]: ", period_);
    period_.value() = 5.0;
  }

  joint_cmd_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/head_controller/joint_trajectory", 100);
  joint_cmd_pub_->on_activate();

  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 100,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "head_1_joint") {
          phase_ = msg->position[i];
          break;
        }
      }
    });
}

void
Pan::halt()
{
  joint_cmd_pub_->on_deactivate();
}

double
Pan::get_joint_yaw(double period, double range, double time, double phase)
{
  return range * sin((2 * M_PI / period) * time + phase);
}

BT::NodeStatus
Pan::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
    phase_ = asin(phase_ / joint_range_.value());
    joint_state_sub_ = nullptr;
  }

  trajectory_msgs::msg::JointTrajectory command_msg;
  auto elapsed = node_->now() - start_time_;

  double yaw = get_joint_yaw(period_.value(), joint_range_.value(), elapsed.seconds(), phase_);

  command_msg.joint_names = std::vector<std::string>{"head_1_joint", "head_2_joint"};
  command_msg.points.resize(1);
  command_msg.points[0].positions.resize(2);
  command_msg.points[0].velocities.resize(2);
  command_msg.points[0].accelerations.resize(2);
  command_msg.points[0].positions[0] = yaw;
  command_msg.points[0].positions[1] = 0.0;
  command_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(0.00);
  joint_cmd_pub_->publish(command_msg);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace motion

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<head::Pan>("Pan");
}
