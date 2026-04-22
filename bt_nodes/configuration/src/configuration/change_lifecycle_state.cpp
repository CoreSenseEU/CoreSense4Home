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

#include "configuration/change_lifecycle_state.hpp"

#include <string>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace configuration {

using namespace std::chrono_literals;

ChangeLifecycleState::ChangeLifecycleState(const std::string &xml_tag_name,
                                           const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(xml_tag_name, conf) {
  config().blackboard->get("node", node_);
}

BT::NodeStatus ChangeLifecycleState::tick() {
  RCLCPP_DEBUG(node_->get_logger(), "ChangeLifecycleState ticked");

  if (!getInput("node_name", node_name_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ChangeLifecycleState: missing 'node_name' input");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("deactivate", deactivate_)) {
    deactivate_ = false;
  }

  // Check that state is valid 
  const std::string get_state_service = node_name_ + "/get_state";

  auto state_client =
      node_->create_client<lifecycle_msgs::srv::GetState>(get_state_service);

  if (!state_client->wait_for_service(3s)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ChangeLifecycleState: service '%s' not available",
                 get_state_service.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto state_request =
      std::make_shared<lifecycle_msgs::srv::GetState::Request>();

  auto state_future = state_client->async_send_request(state_request);

  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                         state_future, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ChangeLifecycleState: failed to call '%s'",
                 get_state_service.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto current_state = state_future.get()->current_state.id;
  if (!deactivate_ &&
      current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_INFO(node_->get_logger(),
                "Node '%s' already ACTIVE, skipping transition",
                node_name_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  if (deactivate_ &&
      current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_INFO(node_->get_logger(),
                "Node '%s' already INACTIVE, skipping transition",
                node_name_.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  // If state not valid, change it
  const std::string service_name = node_name_ + "/change_state";

  auto client =
      node_->create_client<lifecycle_msgs::srv::ChangeState>(service_name);

  if (!client->wait_for_service(3s)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ChangeLifecycleState: service '%s' not available",
                 service_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id =
      deactivate_ ? lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE
                  : lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

  auto future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                         future, 5s) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ChangeLifecycleState: failed to call service '%s'",
                 service_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!future.get()->success) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ChangeLifecycleState: transition failed for node '%s'",
                 node_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
              "ChangeLifecycleState: node '%s' successfully %s",
              node_name_.c_str(), deactivate_ ? "deactivated" : "activated");
  return BT::NodeStatus::SUCCESS;
}

void ChangeLifecycleState::halt() {
  RCLCPP_DEBUG(node_->get_logger(), "ChangeLifecycleState halted");
}

} // namespace configuration

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<configuration::ChangeLifecycleState>(
      "ChangeLifecycleState");
}