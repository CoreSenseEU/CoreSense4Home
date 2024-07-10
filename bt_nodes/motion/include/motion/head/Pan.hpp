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

#ifndef HEAD__PAN_HPP_
#define HEAD__PAN_HPP_

#include <cmath>
#include <iostream>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace head
{

class Pan : public BT::ActionNodeBase
{
public:
  explicit Pan(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<double>("range"),     // in degrees
        BT::InputPort<double>("period"),    // in seconds
        BT::InputPort<double>("pitch_angle") // in degrees
      });
  }

private:
  // std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  rclcpp::Time start_time_;
  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    joint_cmd_pub_;
  double yaw_limit_{1.3};
  double pitch_limit_{0.92};
  double pitch_{0.0};
  std::vector<double> yaw_positions_{0.0, 0.7, 0.7, 0.7, 0.7, -0.7, -0.7, 0.0};
  std::vector<double> pitch_positions_{0.0, 0.0, 0.3, -0.3, 0.3, 0.3, -0.3, 0.0};
  std::vector<double> times_from_start_{0.1, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  double joint_range_, period_;
  double pitch_angle_ = 0.0;
  double phase_;

  int current_position_{0};

  double get_joint_yaw(double period, double range, double time, double phase);
};

}  // namespace head

#endif  // HEAD__PAN_HPP_
