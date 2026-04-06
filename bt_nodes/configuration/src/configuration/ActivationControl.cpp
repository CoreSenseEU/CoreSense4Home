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

#include "configuration/ActivationControl.hpp"

namespace configuration
{

ActivationControl::ActivationControl(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
}

BT::NodeStatus ActivationControl::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "ActivationControl ticked");

  // Read inputs
  if (!getInput("deactivate", deactivate_)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "ActivationControl: missing 'deactivate' input, defaulting to false");
    deactivate_ = false;
  }
  if (!getInput("node_name", node_name_)) {
    RCLCPP_ERROR(node_->get_logger(), "ActivationControl: missing 'node_name' input");
    return BT::NodeStatus::FAILURE;
  }

  if (deactivate_) {
    RCLCPP_INFO(
      node_->get_logger(), "ActivationControl: removing activation for %s",
      node_name_.c_str());
    node_->remove_activation(node_name_);
  } else {
    RCLCPP_INFO(
      node_->get_logger(), "ActivationControl: adding activation for %s",
      node_name_.c_str());
    node_->add_activation(node_name_);
  }

  return BT::NodeStatus::SUCCESS;
}

void ActivationControl::halt()
{
  RCLCPP_DEBUG(node_->get_logger(), "ActivationControl halted");
}

}  // namespace configuration

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<configuration::ActivationControl>("ActivationControl");
}
