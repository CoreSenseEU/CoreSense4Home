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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "navigation_system_interfaces/srv/set_truncate_distance.hpp"
#include "motion/navigation/utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

// #include "rclcpp/rclcpp.hpp"
// #include "nav2_msgs/action/compute_path_to_pose.hpp"
// #include "nav2_msgs/action/follow_path.hpp"
// #include "nav2_util/geometry_utils.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"

namespace navigation
{

class MoveTo
  : public motion::BtActionNode<nav2_msgs::action::NavigateToPose,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit MoveTo(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<double>("distance_tolerance"), BT::InputPort<std::string>("tf_frame"),
        BT::InputPort<bool>("will_finish")});
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  double distance_tolerance_;
  std::string tf_frame_, xml_path_;
  geometry_msgs::msg::PoseStamped pose_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  bool will_finish_ = true;
  rclcpp::Client<navigation_system_interfaces::srv::SetTruncateDistance>::SharedPtr
    set_truncate_distance_client_;
  // std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>> compute_action_client_;
  // std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::FollowPath>> follow_action_client_;
  // rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult path_result_;
  // rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult follow_result_;
  // rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr path_goal_handle_;
  // rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr follow_goal_handle_;
  // bool path_result_available_, goal_send_ {false};
};

}  // namespace navigation

#endif  // NAVIGATION__MOVE_TO_HPP_
