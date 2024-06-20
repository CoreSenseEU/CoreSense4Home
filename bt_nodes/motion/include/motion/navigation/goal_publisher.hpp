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

#ifndef NAVIGATION__GOAL_PUBLISHER_HPP_
#define NAVIGATION__GOAL_PUBLISHER_HPP_

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "navigation_system_interfaces/srv/set_mode.hpp"
#include "navigation_system_interfaces/srv/set_truncate_distance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

namespace navigation
{

class GoalPublisher : public BT::ActionNodeBase
{
public:
  explicit GoalPublisher(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("camera_frame"),
        BT::InputPort<double>("distance_tolerance"),
        BT::InputPort<std::string>("frame_to_follow"),
      });
  }

private:

  void goal_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
  geometry_msgs::msg::PoseStamped get_goal_pose(
    const double & distance_to_substract,
    const geometry_msgs::msg::TransformStamped & goal_transform);
  BT::NodeStatus on_idle();
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>> client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr entity_pose_pub_;
  rclcpp::Client<navigation_system_interfaces::srv::SetTruncateDistance>::SharedPtr
    set_truncate_distance_client_;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr>
   goal_handle_;

  std::string camera_frame_, frame_to_follow_, xml_path_;
  double distance_tolerance_, substracted_distance_ = 0.6, distance_to_entity_ = 0.0;
  double x_axis_max_, x_axis_min_, y_axis_max_, y_axis_min_;
  bool is_goal_sent_ = false;
  //   double magnitude;
  // double scale;
  geometry_msgs::msg::TransformStamped entity_transform_;
  geometry_msgs::msg::TransformStamped robot_direction_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::PoseWithCovarianceStamped current_pos_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr sub_goal_status_;

};

}  // namespace navigation

#endif  // NAVIGATION__GOAL_PUBLISHER_HPP_
