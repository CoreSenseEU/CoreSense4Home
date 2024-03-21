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

#include "motion/navigation/LookAt.hpp"
#include <math.h>

namespace navigation
{

LookAt::LookAt(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  action_client_ = rclcpp_action::create_client<TrajectoryAction>(
    node_, "head_controller/follow_joint_trajectory");

  if (!action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
  }
}

BT::NodeStatus
LookAt::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_INFO(node_->get_logger(), "LookAt idle");
    config().blackboard->get("tf_buffer", tf_buffer_);
    return BT::NodeStatus::RUNNING;
  }

  getInput("tf_frames", tf_frames_);
  getInput("tf_frame", tf_frame_);

  if (tf_frames_.size() == 0 && tf_frame_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No goal frame provided");
    return BT::NodeStatus::FAILURE;
  }

  while(!tf_buffer_->canTransform("head_1_link", tf_frame_, tf2::TimePointZero) &&
          rclcpp::ok() &&
          !tf_buffer_->canTransform("head_2_link", tf_frame_, tf2::TimePointZero))
  {
    RCLCPP_INFO(
      node_->get_logger(), "Waiting for transform from head to %s",
      tf_frame_.c_str());
    rclcpp::spin_some(node_->get_node_base_interface());
    return BT::NodeStatus::RUNNING;
  }

  try {
    head_1_transform = tf_buffer_->lookupTransform(
      "head_1_link", tf_frame_,
      tf2::TimePointZero);
    head_2_transform = tf_buffer_->lookupTransform(
      "head_2_link", tf_frame_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform head to %s: %s",
        tf_frame_.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  yaw_ = atan2(head_1_transform.transform.translation.y, head_1_transform.transform.translation.x);
  pitch_ = atan2(head_2_transform.transform.translation.y, head_2_transform.transform.translation.x);
  
  RCLCPP_INFO(node_->get_logger(), "LookAt yaw: %f, pitch: %f", yaw_, pitch_);

  goal_msg.trajectory.joint_names = std::vector<std::string>{"head_1_joint", "head_2_joint"};
  point.positions = std::vector<double>{yaw_, pitch_};
  point.time_from_start = rclcpp::Duration::from_seconds(0.5);
  goal_msg.trajectory.points.push_back(point);

  action_client_->async_send_goal(goal_msg);

  RCLCPP_INFO(node_->get_logger(), "Sending goal to head controller");

  rclcpp::spin_some(node_);
  return BT::NodeStatus::SUCCESS;
}

void
LookAt::halt()
{
  RCLCPP_INFO(node_->get_logger(), "LookAt halted");
}


}  // namespace navigation

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigation::LookAt>("LookAt");
}
