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
#include "std_msgs/msg/string.hpp"
#include "whisper_msgs/action/stt.hpp"

namespace dialog
{

using namespace std::chrono_literals;
using namespace std::placeholders;

Listen::Listen(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: dialog::BtActionNode<whisper_msgs::action::STT, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
  publisher_start_ = node_->create_publisher<std_msgs::msg::Int8>("dialog_action", 10);
  publisher_text_ = node_->create_publisher<std_msgs::msg::String>("/whisper/text", 10);
}

void Listen::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "Listen ticked");
  std::string text_;
  goal_ = whisper_msgs::action::STT::Goal();
  auto msg_dialog_action = std_msgs::msg::Int8();

  msg_dialog_action.data = 0;

  publisher_start_->publish(msg_dialog_action);
}

BT::NodeStatus Listen::on_success()
{
  auto text = result_.result->transcription.text;

  // Trim leading and trailing whitespace
  auto start = text.find_first_not_of(" \t\n\r");
  if (start == std::string::npos) {
    RCLCPP_WARN(node_->get_logger(), "[Listen] Transcription is empty or whitespace-only");
    return BT::NodeStatus::FAILURE;
  }
  text = text.substr(start, text.find_last_not_of(" \t\n\r") - start + 1);

  RCLCPP_INFO(node_->get_logger(), "[Listen] Heard: \"%s\"", text.c_str());

  // Publish the transcribed text
  auto msg = std_msgs::msg::String();
  msg.data = text;
  publisher_text_->publish(msg);

  setOutput("listen_text", text);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace dialog
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<dialog::Listen>(name, "whisper/listen", config);
  };

  factory.registerBuilder<dialog::Listen>("Listen", builder);
}
