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

#include "configuration/SetWp.hpp"

namespace configuration
{

using namespace std::chrono_literals;
using namespace std::placeholders;

SetWp::SetWp(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  node_->declare_parameter("waypoints_names", std::vector<std::string>());
  node_->get_parameter("waypoints_names", wp_names_);
}

BT::NodeStatus
SetWp::tick()
{
  for (auto wp : wp_names_) {
    node_->declare_parameter("waypoints." + wp, std::vector<double>());
    std::vector<double> wp_pos;
    node_->get_parameter("waypoints." + wp, wp_pos);
    config().blackboard->set(wp, wp_pos);
  }

  RCLCPP_INFO(node_->get_logger(), "SetWp ticked");

  return BT::NodeStatus::SUCCESS;
}

void
SetWp::halt()
{
  RCLCPP_INFO(node_->get_logger(), "SetWp halted");
}

}  // namespace configuration

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<configuration::SetWp>("SetWp");
}
