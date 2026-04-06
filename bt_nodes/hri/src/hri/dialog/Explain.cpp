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

#include "hri/dialog/Explain.hpp"

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "explainability_msgs/action/generate_explanation.hpp"

namespace dialog
{

using namespace std::chrono_literals;

Explain::Explain(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: dialog::BtActionNode<
    explainability_msgs::action::GenerateExplanation,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(xml_tag_name, action_name, conf)
{
}

void Explain::on_tick()
{
  std::string question;
  getInput("question", question);

  bool auto_triggered = true;
  getInput("auto_triggered", auto_triggered);

  RCLCPP_INFO(
    node_->get_logger(),
    "[Explain] Requesting explanation for node '%s' (auto_triggered=%s)",
    question.c_str(), auto_triggered ? "true" : "false");

  goal_.question = question;
  goal_.auto_triggered = auto_triggered;
}

BT::NodeStatus Explain::on_success()
{
  const std::string explanation = result_.result->explanation;

  if (explanation.empty()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[Explain] Received empty explanation from server");
    setOutput("explanation", std::string("I am not sure what went wrong."));
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[Explain] Explanation received: \"%s\"", explanation.c_str());

  setOutput("explanation", explanation);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace dialog

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<dialog::Explain>(name, "/generate_explanation", config);
  };

  factory.registerBuilder<dialog::Explain>("Explain", builder);
}
