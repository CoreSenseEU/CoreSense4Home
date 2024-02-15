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

#include "motion/navigation/MoveTo.hpp"

namespace navigation
{

MoveTo::MoveTo(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  getInput("distance_tolerance", distance_tolerance_);

  std::string tf_frame;
  getInput("tf_frame", tf_frame);

  config().blackboard->get("entrance", pose_);
}

BT::NodeStatus
MoveTo::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    RCLCPP_INFO(node_->get_logger(), "MoveTo ticked");
    if (!compute_path()) { return BT::NodeStatus::FAILURE;}
  }

  if (!path_result_available_) {
    return BT::NodeStatus::RUNNING;
  }

  if (!goal_send_) {
    truncate_path();
    follow_path();
    goal_send_ = true;
  }
  

  return BT::NodeStatus::SUCCESS;
}

void
MoveTo::halt()
{
  RCLCPP_INFO(node_->get_logger(), "MoveTo halted");
}

bool
MoveTo::compute_path()
{
  auto path_msg = nav2_msgs::action::ComputePathToPose::Goal();
  path_msg.goal = pose_;
  path_msg.use_start = false;

  compute_action_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(node_, "compute_path");

  if (!compute_action_client_->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(node_->get_logger(), "Compute Path Action server not available after waiting");
    return false;
  }

  path_result_available_ = false;
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();

  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult & result) {
        if (this->path_goal_handle_->get_goal_id() == result.goal_id) {
          path_result_available_ = true;
          path_result_ = result;
          RCLCPP_INFO(node_->get_logger(), "Goal result received");
        }
      };

  auto future_goal_handle = compute_action_client_->async_send_goal(path_msg, send_goal_options);

  if (rclcpp::spin_until_future_complete(node_, future_goal_handle) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "send goal call failed :(");
    return false;
  }

  path_goal_handle_ = future_goal_handle.get();
  if (!path_goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return false;
  }

  return true;
}

bool
MoveTo::truncate_path()
{
  geometry_msgs::msg::PoseStamped final_pose = path_result_.result->path.poses.back();

  double distance_to_goal = nav2_util::geometry_utils::euclidean_distance(
    path_result_.result->path.poses.back(), final_pose);

  while (distance_to_goal < distance_tolerance_ && path_result_.result->path.poses.size() > 2) {
    path_result_.result->path.poses.pop_back();
    distance_to_goal = nav2_util::geometry_utils::euclidean_distance(
      path_result_.result->path.poses.back(), final_pose);
  }

  double dx = final_pose.pose.position.x - path_result_.result->path.poses.back().pose.position.x;
  double dy = final_pose.pose.position.y - path_result_.result->path.poses.back().pose.position.y;

  double final_angle = atan2(dy, dx);

  if (std::isnan(final_angle) || std::isinf(final_angle)) {
    RCLCPP_WARN(
      config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
      "Final angle is not valid while truncating path. Setting to 0.0");
    final_angle = 0.0;
  }

  path_result_.result->path.poses.back().pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(
    final_angle);

  return true;
}

bool
MoveTo::follow_path()
{
  auto path_msg = nav2_msgs::action::FollowPath::Goal();
  path_msg.path = path_result_.result->path;

  follow_action_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(node_, "follow_path");

  if (!follow_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "Follow Action server not available after waiting");
    return false;
  }

  path_result_available_ = false;
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowPath>::SendGoalOptions();

  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult & result) {
        if (this->follow_goal_handle_->get_goal_id() == result.goal_id) {
          path_result_available_ = true;
          follow_result_ = result;
          RCLCPP_INFO(node_->get_logger(), "Goal result received");
        }
      };

  auto future_goal_handle = follow_action_client_->async_send_goal(path_msg, send_goal_options);

  if (rclcpp::spin_until_future_complete(node_, future_goal_handle) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "send goal call failed :(");
    return false;
  }

  follow_goal_handle_ = future_goal_handle.get();
  if (!follow_goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return false;
  }

  return true;
}

}  // namespace navigation

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navigation::MoveTo>("MoveTo");
}