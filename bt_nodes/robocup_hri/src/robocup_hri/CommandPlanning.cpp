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

#include "robocup_hri/CommandPlanning.hpp"

#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "std_msgs/msg/int8.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace robocup_hri
{

using namespace std::chrono_literals;
using namespace std::placeholders;
using json = nlohmann::json;

std::string joinActions(std::vector<std::string> actions)
{
  if (actions.empty()) {
    return "";
  }

  if (actions.size() == 1) {
    return actions[0];
  }

  std::ostringstream oss;
  for (size_t i = 0; i < actions.size(); ++i) {
    if (i == actions.size() - 1) {
      oss << "and " << actions[i];
    } else {
      oss << actions[i];
      if (i != actions.size() - 2) {
        oss << ", ";
      } else {
        oss << " ";
      }
    }
  }

  return oss.str();
}

CommandPlanning::CommandPlanning(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: robocup_hri::BtServiceNode<gpsr_msgs::srv::GeneratePlan,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
  config().blackboard->get("node", node_);
  publisher_start_ = node_->create_publisher<std_msgs::msg::Int8>("/dialog_action", 10);
}


void CommandPlanning::on_tick()
{
  RCLCPP_INFO(node_->get_logger(), "CommandPlanning ticked");

  std::string bt_ = "";
  std::string actions_ = "";

  getInput("command", command_);

  auto msg_dialog_action = std_msgs::msg::Int8();
  msg_dialog_action.data = 2;
  publisher_start_->publish(msg_dialog_action);

  request_->command = command_;
}

void CommandPlanning::on_result()
{
  RCLCPP_INFO(node_->get_logger(), "CommandPlanning onResult");

  auto bt_ = result_.bt_xml;
  auto actions_ = joinActions(result_.action_list);

  setOutput("bt_value", bt_);
  setOutput("actions", actions_);

  setStatus(BT::NodeStatus::SUCCESS);
}

} // namespace robocup_hri


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<robocup_hri::CommandPlanning>(
        name, "gpsr_planning", config);
    };

  factory.registerBuilder<robocup_hri::CommandPlanning>("CommandPlanning", builder);
}
