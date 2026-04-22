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

#include "hri/UpdateLora.hpp"

#include <algorithm>
#include <filesystem>
#include <string>

namespace hri
{

using namespace std::chrono_literals;

UpdateLora::UpdateLora(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  node_ = config().blackboard->get<
    std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode>>("node");

  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  list_client_ = node_->create_client<llama_msgs::srv::ListLoRAs>(
    "/llama/list_loras", rmw_qos_profile_services_default, callback_group_);
  update_client_ = node_->create_client<llama_msgs::srv::UpdateLoRAs>(
    "/llama/update_loras", rmw_qos_profile_services_default, callback_group_);

  RCLCPP_INFO(node_->get_logger(), "UpdateLora BT node initialized");
}

BT::NodeStatus UpdateLora::tick()
{
  std::string lora_name;
  float lora_scale;

  if (!getInput("lora_name", lora_name) || !getInput("lora_scale", lora_scale)) {
    RCLCPP_ERROR(node_->get_logger(), "UpdateLora: missing required inputs");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(), "UpdateLora: looking for LoRA '%s' with scale %.2f",
    lora_name.c_str(), lora_scale);

  // 1) Call ListLoRAs
  if (!list_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(node_->get_logger(), "UpdateLora: list_loras service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto list_request = std::make_shared<llama_msgs::srv::ListLoRAs::Request>();
  auto list_future = list_client_->async_send_request(list_request).share();

  auto list_ret = callback_group_executor_.spin_until_future_complete(list_future);
  if (list_ret != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "UpdateLora: list_loras service call failed");
    return BT::NodeStatus::FAILURE;
  }

  auto list_response = list_future.get();

  // 2) Find the LoRA whose path filename matches lora_name
  bool found = false;
  auto loras = list_response->loras;

  for (auto & lora : loras) {
    std::string filename = std::filesystem::path(lora.path).stem().string();
    RCLCPP_INFO(
        node_->get_logger(), "UpdateLora: LoRA id=%d path='%s', setting scale=%.2f",
        lora.id, filename.c_str(), lora_scale);
    if (filename == lora_name) {
      lora.scale = lora_scale;
      found = true;
      RCLCPP_INFO(
        node_->get_logger(), "UpdateLora: matched LoRA id=%d path='%s', setting scale=%.2f",
        lora.id, lora.path.c_str(), lora_scale);
      break;
    }
  }

  if (!found) {
    RCLCPP_ERROR(
      node_->get_logger(), "UpdateLora: no LoRA found matching name '%s'",
      lora_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 3) Call UpdateLoRAs with the full list (modified scale)
  if (!update_client_->wait_for_service(5s)) {
    RCLCPP_ERROR(node_->get_logger(), "UpdateLora: update_loras service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto update_request = std::make_shared<llama_msgs::srv::UpdateLoRAs::Request>();
  update_request->loras = loras;
  auto update_future = update_client_->async_send_request(update_request).share();

  auto update_ret = callback_group_executor_.spin_until_future_complete(update_future);
  if (update_ret != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "UpdateLora: update_loras service call failed");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "UpdateLora: successfully updated LoRA scale");
  return BT::NodeStatus::SUCCESS;
}

void UpdateLora::halt()
{
  setStatus(BT::NodeStatus::IDLE);
}

}  // namespace hri


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hri::UpdateLora>("UpdateLora");
}