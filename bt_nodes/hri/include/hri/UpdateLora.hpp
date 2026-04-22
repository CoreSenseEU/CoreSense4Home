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

#ifndef HRI__UPDATE_LORA_HPP_
#define HRI__UPDATE_LORA_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "llama_msgs/srv/list_lo_r_as.hpp"
#include "llama_msgs/srv/update_lo_r_as.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace hri
{

class UpdateLora : public BT::ActionNodeBase
{
public:
  explicit UpdateLora(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void halt() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("lora_name", "Name to match against the LoRA filename"),
        BT::InputPort<float>("lora_scale", "New scale value for the matched LoRA"),
      });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;

  rclcpp::Client<llama_msgs::srv::ListLoRAs>::SharedPtr list_client_;
  rclcpp::Client<llama_msgs::srv::UpdateLoRAs>::SharedPtr update_client_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

}  // namespace hri

#endif  // HRI__UPDATE_LORA_HPP_