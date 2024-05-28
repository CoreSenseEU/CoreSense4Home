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

#include "hri/CommandPlanning.hpp"

#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "std_msgs/msg/int8.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "gpsr_msgs/srv/generate_plan.hpp"

namespace hri {

using namespace std::chrono_literals;
using namespace std::placeholders;
using json = nlohmann::json;

CommandPlanning::CommandPlanning(const std::string &xml_tag_name,
                                 const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(xml_tag_name, conf) {
  config().blackboard->get("node", node_);

  this->publisher_start_ =
      node_->create_publisher<std_msgs::msg::Int8>("dialog_action", 10);

  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_executor_.add_callback_group(callback_group_,
                                        node_->get_node_base_interface());

  this->generate_plan_client_ =
      node_->create_client<gpsr_msgs::srv::GeneratePlan>(
          "gpsr_planning", rmw_qos_profile_services_default, callback_group_);
}

void CommandPlanning::halt() {
  RCLCPP_INFO(node_->get_logger(), "CommandPlanning halted");
}

BT::NodeStatus CommandPlanning::tick() {
  RCLCPP_DEBUG(node_->get_logger(), "CommandPlanning ticked");

  std::string bt_ = "";
  std::string actions_ = "";

  if (!generate_plan_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_->get_logger(), "Waiting for action server to be up...");
    return BT::NodeStatus::RUNNING;
  }

  getInput("command", command_);

  auto msg_dialog_action = std_msgs::msg::Int8();

  msg_dialog_action.data = 2;

  publisher_start_->publish(msg_dialog_action);

  auto request = std::make_shared<gpsr_msgs::srv::GeneratePlan::Request>();
  request->command = command_;

  auto result = this->generate_plan_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    bt_ = result.get()->bt_xml;
  } else {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("bt_value", bt_);
  setOutput("actions", actions_);
  return BT::NodeStatus::SUCCESS;
}

} // namespace hri
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {

  factory.registerNodeType<hri::CommandPlanning>("CommandPlanning");
}
