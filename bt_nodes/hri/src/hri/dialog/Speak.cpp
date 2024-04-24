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

#include <cstdint>
#include <string>
#include <utility>

#include "audio_common_msgs/action/tts.hpp"
#include "hri/dialog/Speak.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace dialog
{
using namespace std::chrono_literals;
using namespace std::placeholders;

Speak::Speak(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: dialog::BtActionNode<audio_common_msgs::action::TTS>(xml_tag_name,
    action_name, conf)
{
  config().blackboard->get("node", node_);

  this->publisher_ =
    node_->create_publisher<std_msgs::msg::String>("say_text", 10);
  this->publisher_start_ =
    node_->create_publisher<std_msgs::msg::Int8>("dialog_action", 10);

  this->publisher_->on_activate();
  this->publisher_start_->on_activate();
}

void Speak::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "Speak ticked");
  std::string text_;

  getInput("say_text", text_);
  std::string param_;

  getInput("param", param_);

  if (param_.length() > 0) {
    goal_.text = text_ + " " + param_ + "?";
  } else {
    goal_.text = text_;
  }

  auto msg = std_msgs::msg::String();
  auto msg_dialog_action = std_msgs::msg::Int8();

  msg.data = goal_.text;
  msg_dialog_action.data = 1;

  this->publisher_->publish(msg);
  this->publisher_start_->publish(msg_dialog_action);
}

BT::NodeStatus Speak::on_success() {return BT::NodeStatus::SUCCESS;}
} // namespace dialog
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string & name,
      const BT::NodeConfiguration & config) {
      return std::make_unique<dialog::Speak>(name, "/say", config);
    };

  factory.registerBuilder<dialog::Speak>("Speak", builder);
}
