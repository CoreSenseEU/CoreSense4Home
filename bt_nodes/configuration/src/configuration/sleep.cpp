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

#include "configuration/sleep.hpp"

namespace configuration
{

Sleep::Sleep(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
}

BT::NodeStatus Sleep::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[Sleep] ticked");

    getInput("time", time_);

  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = std::chrono::high_resolution_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time_).count();
  if (elapsed_time < time_) {
    return BT::NodeStatus::RUNNING;
  } else {
    return BT::NodeStatus::SUCCESS;
  }
}

void Sleep::halt()
{
  RCLCPP_INFO(node_->get_logger(), "[Sleep] halted");
}

}  // namespace navigation

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<configuration::Sleep>("Sleep");
}
