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

#ifndef NAVIGATION__MOVE_TO_HPP_
#define NAVIGATION__MOVE_TO_HPP_

#include <string>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace navigation
{

class MoveTo : public BT::ActionNodeBase
{
public:
  explicit MoveTo(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<double>("distance_tolerance"),
        BT::InputPort<std::string>("tf_frame"),
      });
  }

private:
  bool create_and_send_goal();

  rclcpp::Node::SharedPtr node_;
  double distance_tolerance_;
  geometry_msgs::msg::PoseStamped pose_;
  std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>> action_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
  bool goal_result_available_{false};
};

}  // namespace navigation

#endif  // NAVIGATION__MOVE_TO_HPP_
