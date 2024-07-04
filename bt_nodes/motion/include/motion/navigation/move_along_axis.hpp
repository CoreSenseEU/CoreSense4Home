// Copyright 2024 Intelligent Robotics Lab Gentlebots
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

#ifndef NAVIGATION__MOVE_ALONG_AXIS_HPP_
#define NAVIGATION__MOVE_ALONG_AXIS_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"


namespace navigation
{

class MoveAlongAxis : public BT::ActionNodeBase
{
public:
  explicit MoveAlongAxis(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({
      BT::InputPort<std::string>("cmd_vel_topic"),
      BT::InputPort<double>("speed"),
      BT::InputPort<double>("distance"),
      BT::InputPort<std::string>("axis")
    });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  rclcpp::Time start_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  double speed_, distance_;
  std::string cmd_vel_topic_{"/cmd_vel"}, axis_{"z"};
};

}  // namespace navigation

#endif  // NAVIGATION__MOVE_ALONG_AXIS_HPP_