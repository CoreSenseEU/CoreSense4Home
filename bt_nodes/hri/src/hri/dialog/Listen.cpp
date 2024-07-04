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

#include "hri/dialog/Listen.hpp"

#include <cstdint>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "std_msgs/msg/int8.hpp"
#include "whisper_msgs/action/stt.hpp"

namespace dialog
{

using namespace std::chrono_literals;
using namespace std::placeholders;

Listen::Listen(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(
    xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  publisher_start_ = node_->create_publisher<std_msgs::msg::Int8>("dialog_action", 10);
  publisher_start_->on_activate();

  client_ = rclcpp_action::create_client<whisper_msgs::action::STT>(
    node_, "whisper/listen");

}

void Listen::halt()
{
  RCLCPP_INFO(node_->get_logger(), "Listen halted");
}

BT::NodeStatus Listen::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "Listen ticked");

  auto msg_dialog_action = std_msgs::msg::Int8();

  msg_dialog_action.data = 0;

  publisher_start_->publish(msg_dialog_action);

  if (status() == BT::NodeStatus::IDLE || !is_goal_sent_) {
    return on_idle();
  }

  if (text_.size() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "whisper not listen anything");
    return BT::NodeStatus::FAILURE;
  }

  fprintf(stderr, "%s\n", text_.c_str());

  setOutput("listen_text", text_);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Listen::on_idle()
{

  auto goal = whisper_msgs::action::STT::Goal();

  RCLCPP_INFO(node_->get_logger(), "Sending goal");

  auto future_goal_handle = client_->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(
      node_->get_node_base_interface(),
      future_goal_handle) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {

    RCLCPP_ERROR(node_->get_logger(), "send_goal failed");
    is_goal_sent_ = false;
    return BT::NodeStatus::RUNNING;
  }

  auto goal_handle = future_goal_handle.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return BT::NodeStatus::RUNNING;
  }

  // Wait for the server to be done with the goal
  auto result_future = client_->async_get_result(goal_handle);

  RCLCPP_INFO(node_->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "get result call failed :(");
    return BT::NodeStatus::RUNNING;
  }

  auto wrapped_result = result_future.get();

  if(wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected");
    return BT::NodeStatus::RUNNING;
  }

  is_goal_sent_ = true;
  text_ = wrapped_result.result->text; 

  return BT::NodeStatus::RUNNING;
}

}  // namespace dialog
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {

  factory.registerNodeType<dialog::Listen>("Listen");
}