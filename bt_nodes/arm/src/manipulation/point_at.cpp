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

#include "arm/manipulation/point_at.hpp"

#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace manipulation
{

using namespace std::chrono_literals;
using namespace std::placeholders;

PointAt::PointAt(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
}

void PointAt::halt()
{
  RCLCPP_INFO(node_->get_logger(), "PointAt halted");
}

BT::NodeStatus PointAt::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "PointAt ticked");

  config().blackboard->get("tf_buffer", tf_buffer_);

  getInput("pose_to_point", pose_to_point_);
  getInput("tf_frame", tf_frame_);
  getInput("base_frame", base_frame_);
  getInput("low_z", low_z_);
  getInput("high_z", high_z_);

  geometry_msgs::msg::Pose target_pose;

  if (tf_frame_.empty() && pose_to_point_) {
    target_pose = pose_to_point_->pose;
  } else if (!tf_frame_.empty()) {
    try {
      transform_ = tf_buffer_->lookupTransform(
        base_frame_, tf_frame_, tf2::TimePointZero, tf2::durationFromSec(2));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        node_->get_logger(), "Could not transform %s to %s: %s", tf_frame_.c_str(),
        base_frame_.c_str(), ex.what());
      return BT::NodeStatus::FAILURE;
    }

    // Calculate pointing pose
    auto angle = std::atan2(
      transform_.transform.translation.y,
      transform_.transform.translation.x);
    double desired_radius = 0.9;
    auto x_point = desired_radius * std::cos(angle);
    auto y_point = desired_radius * std::sin(angle);

    target_pose.position.x = (std::isnan(x_point) || std::isinf(x_point)) ? 0.0 : x_point;
    target_pose.position.y = (std::isnan(y_point) || std::isinf(y_point)) ? 0.0 : y_point;
    target_pose.position.z = (transform_.transform.translation.z < low_z_) ? low_z_ :
      (transform_.transform.translation.z > high_z_) ?
      high_z_ :
      transform_.transform.translation.z;

    tf2::Quaternion orientation(0, 0, 0, 1);
    orientation.setEuler(0, 0, angle);
    target_pose.orientation.x = orientation.x();
    target_pose.orientation.y = orientation.y();
    target_pose.orientation.z = orientation.z();
    target_pose.orientation.w = orientation.w();

    RCLCPP_INFO(
      node_->get_logger(), "Pointing from %s to %s", base_frame_.c_str(),
      transform_.header.frame_id.c_str());
    RCLCPP_INFO(node_->get_logger(), "Pointing at %s", tf_frame_.c_str());
    RCLCPP_INFO(
      node_->get_logger(), "Pointing to %f %f %f", target_pose.position.x,
      target_pose.position.y, target_pose.position.z);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "No pose_to_point or tf_frame provided");
    return BT::NodeStatus::FAILURE;
  }

  // Use MoveIt to plan and execute
  try {
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node_, "arm_torso");

    // Set the target pose
    move_group_interface.setPoseTarget(target_pose);

    // Create a plan to that target pose
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_interface.plan(plan));

    if (success) {
      RCLCPP_INFO(node_->get_logger(), "Plan found, executing...");
      auto execute_result = move_group_interface.execute(plan);
      if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "PointAt execution successful");
        return BT::NodeStatus::SUCCESS;
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Execution failed!");
        return BT::NodeStatus::FAILURE;
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
      return BT::NodeStatus::FAILURE;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node_->get_logger(), "Exception during MoveIt planning: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace manipulation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<manipulation::PointAt>("PointAt");
}
