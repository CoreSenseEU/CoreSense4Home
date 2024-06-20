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

#include "perception/is_entity_moving.hpp"

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "perception_system/PerceptionUtils.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsEntityMoving::IsEntityMoving(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  // tf_buffer_ =
  //   std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  // tf_listener_ =
  //   std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)
}

BT::NodeStatus IsEntityMoving::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    config().blackboard->get("tf_buffer", tf_buffer_);
  }
  getInput("frame", frame_);
  getInput("check_time", check_time_);
  getInput("distance_tolerance", distance_tolerance_);
  getInput("robot_distance_to_person", robot_distance_to_person_);
  // RCLCPP_INFO(node_->get_logger(), "IsEntityMoving ticked");
  RCLCPP_DEBUG(node_->get_logger(), "IsEntityMoving ticked");

  if (has_stoped_ && (node_->now().seconds() - first_time_.seconds()) <= check_time_) {
    RCLCPP_INFO(node_->get_logger(), "IsEntityMoving waiting!!");
    return BT::NodeStatus::SUCCESS;
  }
  geometry_msgs::msg::TransformStamped robot_distance_to_person_msg;
  geometry_msgs::msg::TransformStamped entity_transform_now_msg;
  geometry_msgs::msg::TransformStamped entity_transform_then_msg;

  rclcpp::Time when = node_->get_clock()->now() - rclcpp::Duration(check_time_, 0);

  try {
    entity_transform_now_msg = tf_buffer_->lookupTransform("map", frame_, tf2::TimePointZero);
    robot_distance_to_person_msg = tf_buffer_->lookupTransform("base_footprint", frame_, tf2::TimePointZero);
    entity_transform_then_msg = tf_buffer_->lookupTransform("map", frame_, when, 500ms);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform %s to %s: %s", frame_.c_str(), "map", ex.what());
    has_stoped_ = false;
    return BT::NodeStatus::SUCCESS;  //ASSUME IS MOVING
  }

  auto distance = std::hypot(
    (entity_transform_now_msg.transform.translation.x -
    entity_transform_then_msg.transform.translation.x),
    (entity_transform_now_msg.transform.translation.y -
    entity_transform_then_msg.transform.translation.y));
  auto robot_distance_to_person = std::hypot(
    robot_distance_to_person_msg.transform.translation.x,
    robot_distance_to_person_msg.transform.translation.y);

  RCLCPP_INFO(node_->get_logger(), "Distance: %f", distance);
  RCLCPP_INFO(
    node_->get_logger(), "Distance in X %f",
    entity_transform_now_msg.transform.translation.x -
    entity_transform_then_msg.transform.translation.x);
  RCLCPP_INFO(
    node_->get_logger(), "Distance in Y %f",
    entity_transform_now_msg.transform.translation.y -
    entity_transform_then_msg.transform.translation.y);

  if (distance <= distance_tolerance_ && robot_distance_to_person <= robot_distance_to_person_) {
    has_stoped_ = true;
    first_time_ = node_->now();
    return BT::NodeStatus::FAILURE;
  }
  has_stoped_ = false;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<perception::IsEntityMoving>("IsEntityMoving");
}
