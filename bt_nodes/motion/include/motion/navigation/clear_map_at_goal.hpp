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

#ifndef NAVIGATION__CLEAR_MAP_AT_GOAL_HPP_
#define NAVIGATION__CLEAR_MAP_AT_GOAL_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/clear_costmap_service.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"

// #include "rclcpp/rclcpp.hpp"
// #include "nav2_msgs/action/compute_path_to_pose.hpp"
// #include "nav2_msgs/action/follow_path.hpp"
// #include "nav2_util/geometry_utils.hpp"
// #include "rclcpp_action/rclcpp_action.hpp"

namespace navigation
{

class ClearMapAtGoal : public BT::SyncActionNode 
{
public:
  explicit ClearMapAtGoal(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal", "Original Goal"),
        BT::InputPort<double>("radius", "Radius to clear")
      });
  }

  BT::NodeStatus tick();

private:
  rclcpp::Node::SharedPtr node_;
  double radius_;
  // std::string tf_frame_;
  // geometry_msgs::msg::PoseStamped pose_;
  // tf2::BufferCore tf_buffer_;
  // tf2_ros::TransformListener tf_listener_;
  // bool will_finish_;
  void clearLayerRegion(
  std::shared_ptr<nav2_costmap_2d::CostmapLayer> & costmap, double pose_x, double pose_y, double reset_distance,
  bool invert);

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  // std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>> compute_action_client_;
  // std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::FollowPath>> follow_action_client_;
  // rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult path_result_;
  // rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult follow_result_;
  // rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr path_goal_handle_;
  // rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::SharedPtr follow_goal_handle_;
  // bool path_result_available_, goal_send_ {false};
};

}  // namespace navigation

#endif  // NAVIGATION__CLEAR_MAP_AT_GOAL_HPP_
