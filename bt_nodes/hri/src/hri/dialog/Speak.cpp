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

#include <string>
#include <utility>

#include "audio_common_msgs/action/tts.hpp"
#include "hri/dialog/Speak.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace dialog
{

using namespace std::chrono_literals;
using namespace std::placeholders;

Speak::Speak(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: dialog::BtActionNode<audio_common_msgs::action::TTS>(xml_tag_name,
    action_name, conf) {}

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
