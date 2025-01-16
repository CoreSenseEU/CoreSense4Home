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

#include "robocup_hri/dialog/DialogConfirmation.hpp"

#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "robocup_hri/dialog/DialogConfirmation.hpp"
#include "std_msgs/msg/int8.hpp"
#include "whisper_msgs/action/stt.hpp"

namespace dialog
{

using namespace std::chrono_literals;
using namespace std::placeholders;

DialogConfirmation::DialogConfirmation(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: dialog::BtActionNode<whisper_msgs::action::STT, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
  publisher_start_ = node_->create_publisher<std_msgs::msg::Int8>("dialog_action", 10);
}

void DialogConfirmation::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "DialogConfirmation ticked");
  std::string text_;
  // getInput("prompt", text_);
  goal_ = whisper_msgs::action::STT::Goal();
  auto msg_dialog_action = std_msgs::msg::Int8();

  msg_dialog_action.data = 0;

  publisher_start_->publish(msg_dialog_action);

  // goal_.text = text_;
}

BT::NodeStatus DialogConfirmation::on_success()
{
  fprintf(stderr, "%s\n", result_.result->transcription.text.c_str());

  if (result_.result->transcription.text.size() == 0) {
    return BT::NodeStatus::FAILURE;
  }

  std::transform(
    result_.result->transcription.text.begin(), result_.result->transcription.text.end(), result_.result->transcription.text.begin(),
    [](unsigned char c) {return std::tolower(c);});
  if (result_.result->transcription.text.find("yes") != std::string::npos) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace dialog
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<dialog::DialogConfirmation>(name, "/whisper/listen", config);
    };

  factory.registerBuilder<dialog::DialogConfirmation>("DialogConfirmation", builder);
}
