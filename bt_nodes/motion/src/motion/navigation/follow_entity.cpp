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

#include "motion/navigation/follow_entity.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "motion/navigation/utils.hpp"

namespace navigation
{

using namespace std::chrono_literals;
using namespace std::placeholders;

FollowEntity::FollowEntity(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  entity_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", 10);
  client_ =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");

  sub_pose_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose", qos, std::bind(&FollowEntity::pose_callback, this, _1));

  set_mode_client_ = node_->create_client<navigation_system_interfaces::srv::SetMode>(
    "navigation_system_node/set_mode");
  set_truncate_distance_client_ =
    node_->create_client<navigation_system_interfaces::srv::SetTruncateDistance>(
    "navigation_system_node/set_truncate_distance");
}

void FollowEntity::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pos_ = *msg;
}

void FollowEntity::halt() {RCLCPP_INFO(node_->get_logger(), "FollowEntity halted");}

void FollowEntity::check_robot_inside_map()
{
  if (
    current_pos_.pose.pose.position.x <= x_axis_max_ &&
    current_pos_.pose.pose.position.y <= y_axis_max_ &&
    current_pos_.pose.pose.position.x >= x_axis_min_ &&
    current_pos_.pose.pose.position.y >= y_axis_min_)
  {
  } else {
    auto request = std::make_shared<navigation_system_interfaces::srv::SetMode::Request>();
    request->mode.id = 2;
    auto result = set_mode_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
      if (!result.get()->success) {
        setStatus(BT::NodeStatus::RUNNING);
      }
    } else {
      setStatus(BT::NodeStatus::RUNNING);
    }
  }
  is_goal_sent_ = false;
}

BT::NodeStatus FollowEntity::tick()
{
  // while (!set_mode_client_->wait_for_service(std::chrono::seconds(1)) ||
  //        !set_truncate_distance_client_->wait_for_service(std::chrono::seconds(1))) {
  //   RCLCPP_INFO(node_->get_logger(), "Waiting for action server to be up...");
  //   return BT::NodeStatus::RUNNING;

  // }

  if (status() == BT::NodeStatus::IDLE || !is_goal_sent_) {
    return on_idle();
  }

  // if (current_pos_ != geometry_msgs::msg::PoseWithCovarianceStamped()) {
  //   check_robot_inside_map();
  // }

  while (!tf_buffer_->canTransform("base_footprint", frame_to_follow_, tf2::TimePointZero) &&
    rclcpp::ok())
  {
    RCLCPP_INFO(
      node_->get_logger(), "Waiting for transform from map to %s", frame_to_follow_.c_str());
    rclcpp::spin_some(node_->get_node_base_interface());
    // return BT::NodeStatus::IDLE;
  }

  try {
    entity_transform_ =
      tf_buffer_->lookupTransform("base_footprint", frame_to_follow_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform base_footprint to %s: %s", frame_to_follow_.c_str(),
      ex.what());
    return BT::NodeStatus::RUNNING;
  }

  goal_pose_ = get_goal_pose(substracted_distance_, entity_transform_);
  goal_pose_.header.stamp = node_->now();
  entity_pose_pub_->publish(goal_pose_);

  rclcpp::spin_some(node_->get_node_base_interface());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowEntity::on_idle()
{
  config().blackboard->get("tf_buffer", tf_buffer_);
  RCLCPP_INFO(node_->get_logger(), "FollowEntity ticked IDLE");
  std::string camera_frame, frame_to_follow;
  if (!getInput<std::string>("camera_frame", camera_frame_)) {
    RCLCPP_ERROR(node_->get_logger(), "camera_frame not provided");
    return BT::NodeStatus::RUNNING;
  }
  if (!getInput<std::string>("frame_to_follow", frame_to_follow_)) {
    RCLCPP_ERROR(node_->get_logger(), "frame_to_follow not provided");
    return BT::NodeStatus::RUNNING;
  }
  if (!getInput<double>("distance_tolerance", distance_tolerance_)) {
    RCLCPP_ERROR(node_->get_logger(), "distance_tolerance not provided");
    return BT::NodeStatus::RUNNING;
  }
  if (!getInput<double>("x_axis_max", x_axis_max_)) {
    RCLCPP_ERROR(node_->get_logger(), "x_axis_max not provided");
    return BT::NodeStatus::RUNNING;
  }
  if (!getInput<double>("x_axis_min", x_axis_min_)) {
    RCLCPP_ERROR(node_->get_logger(), "x_axis_min not provided");
    return BT::NodeStatus::RUNNING;
  }
  if (!getInput<double>("y_axis_max", y_axis_max_)) {
    RCLCPP_ERROR(node_->get_logger(), "y_axis_max not provided");
    return BT::NodeStatus::RUNNING;
  }
  if (!getInput<double>("y_axis_min", y_axis_min_)) {
    RCLCPP_ERROR(node_->get_logger(), "y_axis_min not provided");
    return BT::NodeStatus::RUNNING;
  }

  while (!tf_buffer_->canTransform("base_footprint", frame_to_follow_, tf2::TimePointZero) &&
    rclcpp::ok())
  {
    RCLCPP_INFO(
      node_->get_logger(), "Waiting for transform from base_footprint to %s",
      frame_to_follow_.c_str());
    rclcpp::spin_some(node_->get_node_base_interface());
    return BT::NodeStatus::RUNNING;
  }

  try {
    entity_transform_ =
      tf_buffer_->lookupTransform("base_footprint", frame_to_follow_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform base_footprint to %s: %s", frame_to_follow_.c_str(),
      ex.what());
    return BT::NodeStatus::RUNNING;
  }

  goal_pose_ = get_goal_pose(substracted_distance_, entity_transform_);
  auto goal = nav2_msgs::action::NavigateToPose::Goal();

  // xml_path_ = generate_xml_file(dynamic_following_xml, distance_tolerance_);
  auto request =
    std::make_shared<navigation_system_interfaces::srv::SetTruncateDistance::Request>();
  RCLCPP_INFO(node_->get_logger(), "Setting truncate distance to %f", distance_tolerance_);
  request->distance = distance_tolerance_;
  request->xml_content = dynamic_following_xml;

  auto future_request = set_truncate_distance_client_->async_send_request(request).share();
  if (
    rclcpp::spin_until_future_complete(node_, future_request) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node_->get_logger(), "Truncate distance setted");
    auto result = *future_request.get();
    if (!result.success) {
      RCLCPP_INFO(node_->get_logger(), "Truncate distance FAILED calling service");
      return BT::NodeStatus::RUNNING;
    }
    xml_path_ = result.xml_path;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Truncate distance FAILED");
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node_->get_logger(), "Sending goal");

  goal.pose = goal_pose_;
  goal.behavior_tree = xml_path_;

  auto future_goal_handle = client_->async_send_goal(goal);
  if (
    rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future_goal_handle) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "send_goal failed");
    is_goal_sent_ = false;
    return BT::NodeStatus::RUNNING;
  }
  is_goal_sent_ = true;
  return BT::NodeStatus::RUNNING;
}

geometry_msgs::msg::PoseStamped FollowEntity::get_goal_pose(
  const double & distance_to_substract, const geometry_msgs::msg::TransformStamped & goal_transform)
{
  geometry_msgs::msg::PoseStamped goal_pose;

  goal_pose.header.frame_id = "base_footprint";

  double magnitude =
    std::hypot(goal_transform.transform.translation.x, goal_transform.transform.translation.y);
  double scale = (magnitude - distance_to_substract) / magnitude;

  goal_pose.pose.position.x = goal_transform.transform.translation.x * std::max(scale, 0.0);
  goal_pose.pose.position.y = goal_transform.transform.translation.y * std::max(scale, 0.0);

  tf2::Quaternion q;

  if (goal_transform.transform.translation.x == 0 && goal_transform.transform.translation.y == 0) {
    q.setRPY(0, 0, 0);
  } else {
    q.setRPY(
      0, 0,
      std::atan2(goal_transform.transform.translation.y, goal_transform.transform.translation.x));
  }

  goal_pose.pose.orientation = tf2::toMsg(q);
  return goal_pose;
}

}  // namespace navigation

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<navigation::FollowEntity>("FollowEntity");
}
