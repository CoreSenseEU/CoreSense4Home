// Copyright 2024 Intelligent Robotics Lab
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

#include "cs4home_core/CognitiveModule.hpp"

namespace cs4home_core
{

CognitiveModule::CognitiveModule(const rclcpp::NodeOptions & options)
: LifecycleNode("cognitive_module", options)
{
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT CognitiveModule::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  afferent_ = Afferent::make_shared(shared_from_this());
  efferent_ = Efferent::make_shared(shared_from_this());
  core_ = Core::make_shared(shared_from_this());
  meta_ = Meta::make_shared(shared_from_this());
  coupling_ = Coupling::make_shared(shared_from_this());

  if (!afferent_->configure()) {
    RCLCPP_ERROR(get_logger(), "Error configuring afferent at %s", get_name());
    return CallbackReturnT::FAILURE;
  }

  if (!efferent_->configure()) {
    RCLCPP_ERROR(get_logger(), "Error configuring efferent_ at %s", get_name());
    return CallbackReturnT::FAILURE;
  }

  if (!core_->configure()) {
    RCLCPP_ERROR(get_logger(), "Error configuring core_ at %s", get_name());
    return CallbackReturnT::FAILURE;
  }

  if (!meta_->configure()) {
    RCLCPP_ERROR(get_logger(), "Error configuring meta_ at %s", get_name());
    return CallbackReturnT::FAILURE;
  }

  if (!coupling_->configure()) {
    RCLCPP_ERROR(get_logger(), "Error configuring coupling_ at %s", get_name());
    return CallbackReturnT::FAILURE;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT CognitiveModule::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT CognitiveModule::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT CognitiveModule::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT CognitiveModule::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT CognitiveModule::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

}  // namespace cs4home_core
