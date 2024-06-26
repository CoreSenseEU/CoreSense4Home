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

#ifndef NAVIGATION__CONFIGURE_NAVIGATE_BACK_HPP_
#define NAVIGATION__CONFIGURE_NAVIGATE_BACK_HPP_

#include <tf2_ros/buffer.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "navigation_system_interfaces/msg/mode.hpp"
#include "navigation_system_interfaces/srv/set_map.hpp"
#include "navigation_system_interfaces/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "slam_toolbox/srv/save_map.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

namespace navigation
{

//             <Action ID="SetInitialPose" pose="{current_pose}"/>
//             <Action ID="MoveTo" distance_tolerance="0.0" tf_frame="odom"/>

//slam_toolbox/srvs/SaveMap.srv
// navigation_system_interfaces::srv::SetMode
// <navigation_system_interfaces::srv::SetMap

class ConfigureNavigateBack : public BT::ActionNodeBase
{
public:
  explicit ConfigureNavigateBack(
    const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts() {return BT::PortsList({});}

private:
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool save_map(const std::string & map_name);
  bool set_nav2_mode(const int8_t & mode);
  bool set_nav2_map(const std::string & map_path);
  bool set_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);
  BT::NodeStatus on_idle();

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr intial_pose_pub_;
  rclcpp::Client<navigation_system_interfaces::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr save_map_client_;
  rclcpp::Client<navigation_system_interfaces::srv::SetMap>::SharedPtr load_map_client_;
  // tf_buffer:
  // tf_listener:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  geometry_msgs::msg::PoseWithCovarianceStamped current_pos_;
  geometry_msgs::msg::TransformStamped entity_transform_;
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  rclcpp::QoS qos_;

  std::string map_path_;
  std::int8_t mode_;
  bool is_initial_pose_ = false;
  bool is_initial_pose_received_ = false;
  bool is_save_map_ = false;
  bool is_set_mode_ = false;
  bool is_set_map_ = false;
  //   geometry_msgs::msg::PoseStamped get_goal_pose(const double & distance_to_substract, const geometry_msgs::msg::TransformStamped & goal_transform);
  //   void check_robot_inside_map();
  //   BT::NodeStatus on_idle();
  //   std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  //   std::shared_ptr<rclcpp_action::Client<nav2_msgs::action::NavigateToPose>> client_;
  //   rclcpp::Client<navigation_system_interfaces::srv::SetTruncateDistance>::SharedPtr
  //     set_truncate_distance_client_;

  //   std::string camera_frame_, frame_to_follow_, xml_path_;
  //   double distance_tolerance_, substracted_distance_ = 0.6, distance_to_entity_ = 0.0;
  //   double x_axis_max_, x_axis_min_, y_axis_max_, y_axis_min_;
  // //   double magnitude;
  // // double scale;
  //   geometry_msgs::msg::TransformStamped entity_transform_;
  //   geometry_msgs::msg::TransformStamped robot_direction_;
  //   geometry_msgs::msg::PoseStamped goal_pose_;

  //   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  //   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  //   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
};

}  // namespace navigation

#endif  // NAVIGATION__CONFIGURE_NAVIGATE_BACK_HPP_
