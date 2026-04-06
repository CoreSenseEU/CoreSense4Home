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

#include "hri/dialog/get_attended_guest.hpp"
#include <regex>

using std::placeholders::_1;
using namespace std::chrono_literals;


namespace dialog
{

GetAttendedGuest::GetAttendedGuest(
  const std::string & xml_tag_name, const std::string & srv_name,
  const BT::NodeConfiguration & conf)
: hri::BtServiceNode<
    kb_msgs::srv::Query,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(xml_tag_name, srv_name, conf)
{
  config().blackboard->get("node", node_);
}

void GetAttendedGuest::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "[GetAttendedGuest] ticked");
  rclcpp::spin_some(node_->get_node_base_interface());

  // Patterns
  request_->patterns.push_back("robot1 oro:attends ?guest");

  // Vars
  request_->vars.push_back("?guest");

  setStatus(BT::NodeStatus::SUCCESS);
}

void GetAttendedGuest::on_result()
{
  RCLCPP_DEBUG(node_->get_logger(), "[GetAttendedGuest] result received");

  if (!result_.error_msg.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[GetAttendedGuest] error");
    setStatus(BT::NodeStatus::FAILURE);
    return;
  }

  // Find ALL guest matches and pick the one with the highest numeric ID
  std::regex guest_regex("\"guest\"\\s*:\\s*\"guest([0-9]+)\"");
  std::smatch match;
  int max_id = 0;

  auto begin = result_.json.cbegin();
  auto end = result_.json.cend();

  while (std::regex_search(begin, end, match, guest_regex)) {
    int id = std::stoi(match[1]);
    max_id = std::max(max_id, id);
    begin = match.suffix().first;
  }

  guest_id_ = "guest" + std::to_string(max_id);

  if (max_id == 0) {
    RCLCPP_WARN(node_->get_logger(), "[GetAttendedGuest] No attended guest found in the result");
  } else {
    RCLCPP_INFO(
      node_->get_logger(), "[GetAttendedGuest] Found latest guest: %s",
      guest_id_.c_str());
  }

  setOutput("guest_attended", guest_id_);
  setStatus(BT::NodeStatus::SUCCESS);
}


}  // namespace hri

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<dialog::GetAttendedGuest>(
      name, "/kb/query", config);
  };

  factory.registerBuilder<dialog::GetAttendedGuest>("GetAttendedGuest", builder);
}
