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
  sub_map_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", qos, std::bind(&FollowEntity::map_callback, this, _1));
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
  // std::cout << "got pose callback" << std::endl;
}

void FollowEntity::map_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  costmap_ = std::make_shared<nav2_costmap_2d::Costmap2D>(*msg);
  // std::cout << "got map callback" << std::endl;
  sub_map_ = nullptr;
}

void FollowEntity::halt() {RCLCPP_INFO(node_->get_logger(), "FollowEntity halted");}

void FollowEntity::check_robot_inside_map()
{
  int max_x, max_y, current_mx, current_my;
  max_x = costmap_->getSizeInCellsX();
  max_y = costmap_->getSizeInCellsY();
  costmap_->worldToMapNoBounds(
    current_pos_.pose.pose.position.x, current_pos_.pose.pose.position.y, current_mx, current_my);

  if (current_mx <= max_x && current_my <= max_y) {
  } else {
    RCLCPP_INFO(node_->get_logger(), "Robot outside map, stopping");
    auto request = std::make_shared<navigation_system_interfaces::srv::SetMode::Request>();
    request->mode.id = 2;
    auto result = set_mode_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
      if (!result.get()->success) {
        setStatus(BT::NodeStatus::FAILURE);
      }
    } else {
      setStatus(BT::NodeStatus::FAILURE);
    }
  }
}

BT::NodeStatus FollowEntity::tick()
{
  // while (!set_mode_client_->wait_for_service(std::chrono::seconds(1)) ||
  //        !set_truncate_distance_client_->wait_for_service(std::chrono::seconds(1))) {
  //   RCLCPP_INFO(node_->get_logger(), "Waiting for action server to be up...");
  //   return BT::NodeStatus::RUNNING;

  // }

  if (status() == BT::NodeStatus::IDLE) {
    return on_idle();
  }

  // if (costmap_ != nullptr && current_pos_ != geometry_msgs::msg::PoseWithCovarianceStamped()) {
  //   check_robot_inside_map();
  // }

  while (!tf_buffer_->canTransform("map", frame_to_follow_, tf2::TimePointZero) && rclcpp::ok() &&
    !tf_buffer_->canTransform("map", "base_footprint", tf2::TimePointZero))
  {
    RCLCPP_INFO(
      node_->get_logger(), "Waiting for transform from map to %s", frame_to_follow_.c_str());
    rclcpp::spin_some(node_->get_node_base_interface());
  }

  try {
    entity_transform_ = tf_buffer_->lookupTransform("map", frame_to_follow_, tf2::TimePointZero);
    robot_direction_ = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform map to %s: %s", frame_to_follow_.c_str(),
      ex.what());
    return BT::NodeStatus::FAILURE;
  }

  goal_pose_.header.frame_id = "map";
  goal_pose_.header.stamp = node_->now();

  goal_pose_.pose.position.x = entity_transform_.transform.translation.x;
  goal_pose_.pose.position.y = entity_transform_.transform.translation.y;

  tf2::Quaternion current_orientation;
  tf2::Quaternion goal_orientation;

  tf2::fromMsg(entity_transform_.transform.rotation, goal_orientation);
  tf2::fromMsg(robot_direction_.transform.rotation, current_orientation);

  goal_pose_.pose.orientation = tf2::toMsg(goal_orientation - current_orientation);
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
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput<std::string>("frame_to_follow", frame_to_follow_)) {
    RCLCPP_ERROR(node_->get_logger(), "frame_to_follow not provided");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput<double>("distance_tolerance", distance_tolerance_)) {
    RCLCPP_ERROR(node_->get_logger(), "distance_tolerance not provided");
    return BT::NodeStatus::FAILURE;
  }

  while (!tf_buffer_->canTransform("map", frame_to_follow_, tf2::TimePointZero) && rclcpp::ok() &&
    !tf_buffer_->canTransform("map", "base_footprint", tf2::TimePointZero))
  {
    RCLCPP_INFO(
      node_->get_logger(), "Waiting for transform from map to %s", frame_to_follow_.c_str());
    rclcpp::spin_some(node_->get_node_base_interface());
  }

  try {
    entity_transform_ = tf_buffer_->lookupTransform("map", frame_to_follow_, tf2::TimePointZero);
    robot_direction_ = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform map to %s: %s", frame_to_follow_.c_str(),
      ex.what());
    return BT::NodeStatus::FAILURE;
  }

  goal_pose_.header.frame_id = "map";
  goal_pose_.pose.position.x = entity_transform_.transform.translation.x;
  goal_pose_.pose.position.y = entity_transform_.transform.translation.y;

  tf2::Quaternion current_orientation;
  tf2::Quaternion goal_orientation;

  tf2::fromMsg(entity_transform_.transform.rotation, goal_orientation);
  tf2::fromMsg(robot_direction_.transform.rotation, current_orientation);

  goal_pose_.pose.orientation = tf2::toMsg(goal_orientation - current_orientation);

  auto goal = nav2_msgs::action::NavigateToPose::Goal();
  xml_path_ = generate_xml_file(dynamic_following_xml, distance_tolerance_);
  // auto request = std::make_shared<navigation_system_interfaces::srv::SetTruncateDistance::Request>();
  // RCLCPP_INFO(node_->get_logger(), "Setting truncate distance to %f", distance_tolerance_);
  // request->distance = distance_tolerance_;

  // auto future_request = set_truncate_distance_client_->async_send_request(request).share();
  // if (rclcpp::spin_until_future_complete(node_, future_request) ==
  // rclcpp::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_INFO(node_->get_logger(), "Truncate distance setted");
  //   auto result = *future_request.get();
  //   if (!result.success) {
  //     RCLCPP_INFO(node_->get_logger(), "Truncate distance FAILED calling service");
  //     return BT::NodeStatus::FAILURE;
  //   }
  //   xml_path_ = result.xml_path;
  // } else {
  //   RCLCPP_INFO(node_->get_logger(), "Truncate distance FAILED");
  //   return BT::NodeStatus::FAILURE;
  // }

  RCLCPP_INFO(node_->get_logger(), "Sending goal");

  goal.pose = goal_pose_;
  goal.behavior_tree = xml_path_;

  auto future_goal_handle = client_->async_send_goal(goal);
  if (
    rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future_goal_handle) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "send_goal failed");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace navigation

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<navigation::FollowEntity>("FollowEntity");
}