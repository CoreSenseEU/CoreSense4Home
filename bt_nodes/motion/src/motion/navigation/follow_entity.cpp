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

#include <string>
#include <utility>


#include "motion/navigation/utils.hpp"
#include "motion/navigation/follow_entity.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "behaviortree_cpp_v3/behavior_tree.h"


namespace navigation
{

using namespace std::chrono_literals;
using namespace std::placeholders;


FollowEntity::FollowEntity(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  entity_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "goal_update", 10);
  client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    node_, "navigate_to_pose");    
}

void
FollowEntity::halt()
{
  RCLCPP_INFO(node_->get_logger(), "FollowEntity halted");
}

BT::NodeStatus
FollowEntity::tick()
{
  if (status() == BT::NodeStatus::IDLE) 
  {
    return on_idle();
  } 

  while(!tf_buffer_->canTransform("map", frame_to_follow_, tf2::TimePointZero) &&
        rclcpp::ok() &&
        !tf_buffer_->canTransform("map", "base_footprint", tf2::TimePointZero))
  {
    RCLCPP_INFO(
      node_->get_logger(), "Waiting for transform from map to %s",
      frame_to_follow_.c_str());
    rclcpp::spin_some(node_->get_node_base_interface());
  }

  try {
    entity_transform_ = tf_buffer_->lookupTransform(
      "map", frame_to_follow_,
      tf2::TimePointZero);
    robot_direction_ = tf_buffer_->lookupTransform(
      "map", "base_footprint",
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform map to %s: %s",
        frame_to_follow_.c_str(), ex.what());
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

BT::NodeStatus
FollowEntity::on_idle()
{
  config().blackboard->get("tf_buffer", tf_buffer_);
  RCLCPP_INFO(node_->get_logger(), "FollowEntity ticked IDLE");
  std::string camera_frame, frame_to_follow;
  if (!getInput<std::string>("camera_frame", camera_frame_)) 
  {
    RCLCPP_ERROR(node_->get_logger(), "camera_frame not provided");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput<std::string>("frame_to_follow", frame_to_follow_)) 
  {
    RCLCPP_ERROR(node_->get_logger(), "frame_to_follow not provided");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput<double>("distance_tolerance", distance_tolerance_)) 
  {
    RCLCPP_ERROR(node_->get_logger(), "distance_tolerance not provided");
    return BT::NodeStatus::FAILURE;
  }
  
  while(!tf_buffer_->canTransform("map", frame_to_follow_, tf2::TimePointZero) &&
        rclcpp::ok() &&
        !tf_buffer_->canTransform("map", "base_footprint", tf2::TimePointZero))
  {
    RCLCPP_INFO(
      node_->get_logger(), "Waiting for transform from map to %s",
      frame_to_follow_.c_str());
    rclcpp::spin_some(node_->get_node_base_interface());
  }

  try {
    entity_transform_ = tf_buffer_->lookupTransform(
      "map", frame_to_follow_,
      tf2::TimePointZero);
    robot_direction_ = tf_buffer_->lookupTransform(
      "map", "base_footprint",
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform map to %s: %s",
        frame_to_follow_.c_str(), ex.what());
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
  xml_path_ = generate_xml_file(dynamic_following_xml,distance_tolerance_);

  goal.pose = goal_pose_;
  goal.behavior_tree = xml_path_;

  auto future_goal_handle = client_->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(
      node_->get_node_base_interface(), future_goal_handle) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "send_goal failed");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace navigation


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigation::FollowEntity>("FollowEntity");
}
