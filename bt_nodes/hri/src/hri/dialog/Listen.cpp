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

#include "hri/dialog/Listen.hpp"
#include "whisper_msgs/action/stt.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace dialog {

using namespace std::chrono_literals;
using namespace std::placeholders;

Listen::Listen(const std::string &xml_tag_name, const std::string &action_name,
               const BT::NodeConfiguration &conf)
    : dialog::BtActionNode<whisper_msgs::action::STT>(xml_tag_name, action_name,
                                                      conf) {}

void Listen::on_tick() {

  RCLCPP_DEBUG(node_->get_logger(), "Listen ticked");
  std::string text_;
  goal_ = whisper_msgs::action::STT::Goal();
}

BT::NodeStatus Listen::on_success() {
  fprintf(stderr, "%s\n", result_.result->text.c_str());

  if (result_.result->text.size() == 0) {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("listen_text", result_.result->text);
  return BT::NodeStatus::SUCCESS;
}

} // namespace dialog
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration &config) {
    return std::make_unique<dialog::Listen>(name, "whisper/listen", config);
  };

  factory.registerBuilder<dialog::Listen>("Listen", builder);
}
