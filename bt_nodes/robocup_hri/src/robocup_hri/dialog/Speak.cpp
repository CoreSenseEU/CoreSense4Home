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

#include "robocup_hri/dialog/Speak.hpp"

#include <cstdint>
#include <string>
#include <utility>

#include "audio_common_msgs/action/tts.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

namespace dialog
{
using namespace std::chrono_literals;
using namespace std::placeholders;

Speak::Speak(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  this->publisher_ = node_->create_publisher<std_msgs::msg::String>("say_text", 10);
  this->publisher_start_ = node_->create_publisher<std_msgs::msg::Int8>("dialog_action", 10);

  this->publisher_->on_activate();
  this->publisher_start_->on_activate();

  this->client_ = rclcpp_action::create_client<audio_common_msgs::action::TTS>(node_, "/say");
}

void Speak::halt() {RCLCPP_INFO(node_->get_logger(), "Speak halted");}

BT::NodeStatus Speak::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "Speak ticked");
  if (status() == BT::NodeStatus::IDLE || !is_goal_sent_) {
    return on_idle();
  }

  return BT::NodeStatus::RUNNING;
  /* std::string text_;

  getInput("say_text", text_);
  std::string param_;

  getInput("param", param_);

  if (param_.length() > 0) {
    goal_.text = text_ + " " + param_;
  } else {
    goal_.text = text_;
  }

  auto msg = std_msgs::msg::String();
  auto msg_dialog_action = std_msgs::msg::Int8();

  msg.data = goal_.text;
  msg_dialog_action.data = 1;

  this->publisher_->publish(msg);
  this->publisher_start_->publish(msg_dialog_action); */
}

BT::NodeStatus Speak::on_idle()
{
  auto goal = audio_common_msgs::action::TTS::Goal();

  std::string text_;
  getInput("say_text", text_);

  std::string param_;
  getInput("param", param_);

  if (param_.length() > 0) {
    goal.text = text_ + " " + param_;
  } else {
    goal.text = text_;
  }

  auto msg = std_msgs::msg::String();
  auto msg_dialog_action = std_msgs::msg::Int8();

  msg.data = goal.text;
  msg_dialog_action.data = 1;

  this->publisher_->publish(msg);
  this->publisher_start_->publish(msg_dialog_action);

  RCLCPP_INFO(node_->get_logger(), "Sending goal");

  auto future_goal_handle = client_->async_send_goal(goal);
  if (
    rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future_goal_handle) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    /* is_goal_sent_ = true;
    auto result = *future_goal_handle.get();
    text_ = result.result->text; */
    RCLCPP_ERROR(node_->get_logger(), "send_goal failed");
    is_goal_sent_ = false;
    return BT::NodeStatus::FAILURE;
  }

  auto goal_handle = future_goal_handle.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return BT::NodeStatus::FAILURE;
  }

  // Wait for the server to be done with the goal
  auto result_future = client_->async_get_result(goal_handle);

  RCLCPP_INFO(node_->get_logger(), "Waiting for result");
  if (
    rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "get result call failed :(");
    return BT::NodeStatus::FAILURE;
  }

  auto wrapped_result = result_future.get();

  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected");
    return BT::NodeStatus::FAILURE;
  }

  is_goal_sent_ = true;

  return BT::NodeStatus::SUCCESS;
}
}  // namespace dialog
// namespace dialog
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<dialog::Speak>("Speak");
}
