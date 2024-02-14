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
}

BT::NodeStatus
MoveTo::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    std::string tf_frame;
    getInput("tf_frame", tf_frame);
    getInput("distance_tolerance", distance_tolerance_);
    config().blackboard->get(tf_frame, pose_);

    RCLCPP_INFO(node_->get_logger(), "MoveTo ticked");
    if (!create_and_send_goal()) { return BT::NodeStatus::FAILURE;}
  }

  if (!goal_result_available_) {
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::SUCCESS;
}

void
MoveTo::halt()
{
  RCLCPP_INFO(node_->get_logger(), "MoveTo halted");
}

bool
MoveTo::create_and_send_goal()
{
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose = pose_;

  action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");

  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    return false;
  }

  goal_result_available_ = false;
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
        if (this->goal_handle_->get_goal_id() == result.goal_id) {
          goal_result_available_ = true;
          result_ = result;
          RCLCPP_INFO(node_->get_logger(), "Goal result received");
        }
      };

  // TODO: Stop the robot if the goal is under the distance_tolerance

  auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);

  if (rclcpp::spin_until_future_complete(node_, future_goal_handle) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "send goal call failed :(");
    return false;
  }

  goal_handle_ = future_goal_handle.get();
  if (!goal_handle_) {
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
