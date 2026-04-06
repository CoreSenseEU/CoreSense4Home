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

#include "hri/dialog/remove_guest_attended.hpp"
#include <regex>

using std::placeholders::_1;
using namespace std::chrono_literals;


namespace dialog
{

RemoveGuestAttended::RemoveGuestAttended(
  const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  this->kb_publisher_ = node_->create_publisher<std_msgs::msg::String>("/kb/remove_fact", 10);
}

BT::NodeStatus RemoveGuestAttended::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "[RemoveGuestAttended] ticked");

  getInput("guest_attended", guest_attending_id_);

  std_msgs::msg::String fact_msg;

  if (!guest_attending_id_.empty()) {
    fact_msg.data = "robot1 oro:attends " + guest_attending_id_;
    kb_publisher_->publish(fact_msg);
    RCLCPP_INFO(
      node_->get_logger(), "[RemoveGuestAttended] Removing fact: robot1 oro:attends %s",
      guest_attending_id_.c_str());
  } else {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;

}

void RemoveGuestAttended::halt() {}


}  // namespace hri

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{

  factory.registerNodeType<dialog::RemoveGuestAttended>("RemoveGuestAttended");
}
