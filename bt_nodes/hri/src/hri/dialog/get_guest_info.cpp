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

#include "hri/dialog/get_guest_info.hpp"
#include <regex>

using std::placeholders::_1;
using namespace std::chrono_literals;


namespace dialog
{

GetGuestInfo::GetGuestInfo(
  const std::string & xml_tag_name, const std::string & srv_name,
  const BT::NodeConfiguration & conf)
: hri::BtServiceNode<
    kb_msgs::srv::Query,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(xml_tag_name, srv_name, conf)
{
  config().blackboard->get("node", node_);
  this->kb_publisher_= node_->create_publisher<std_msgs::msg::String>("/kb/remove_fact", 10);
}

void GetGuestInfo::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "[GetGuestInfo] ticked");
  rclcpp::spin_some(node_->get_node_base_interface());
  
  // Patterns
  request_->patterns.push_back("robot1 oro:attends ?guest");
  request_->patterns.push_back("?guest oro:hasName ?name");
  request_->patterns.push_back("?guest oro:hasFavoriteDrink ?drink");
  request_->patterns.push_back("?guest oro:description ?desc");

  // Vars
  request_->vars.push_back("?guest");
  request_->vars.push_back("?name");
  request_->vars.push_back("?drink");
  request_->vars.push_back("?desc");


  setStatus(BT::NodeStatus::SUCCESS);
}

void GetGuestInfo::on_result()
{
  RCLCPP_DEBUG(node_->get_logger(), "[GetGuestInfo] result received");

  if (!result_.error_msg.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[GetGuestInfo] error");
    setStatus(BT::NodeStatus::FAILURE);
  }

  std::regex guest_regex("\"guest\"\\s*:\\s*\"([^\"]*)\"");
  std::regex name_regex("\"name\"\\s*:\\s*\"([^\"]*)\"");
  std::regex drink_regex("\"drink\"\\s*:\\s*\"([^\"]*)\"");
  std::regex desc_regex("\"desc\"\\s*:\\s*\"([^\"]*)\"");

  std::smatch match;

  if (std::regex_search(result_.json, match, guest_regex)) {
    guest_id_ = match[1];
  }

  if (std::regex_search(result_.json, match, name_regex)) {
    guest_name_ = match[1];
  }

  if (std::regex_search(result_.json, match, drink_regex)) {
    guest_drink_ = match[1];
  }

  if (std::regex_search(result_.json, match, desc_regex)) {
    guest_description_ = match[1];
  }

  setOutput("guest_name", guest_name_);
  setOutput("guest_drink", guest_drink_);
  setOutput("guest_description", guest_description_);

  RCLCPP_INFO(
    node_->get_logger(), "[GetGuestInfo] Guest info retrieved: Name: %s, Drink: %s, Description: %s",
    guest_name_.c_str(), guest_drink_.c_str(), guest_description_.c_str());

  std_msgs::msg::String fact_msg;

  // Delete attending fact
  if (!guest_id_.empty()) {
    fact_msg.data = "robot1 oro:attends " + guest_id_;
    kb_publisher_->publish(fact_msg);
  }

  setStatus(BT::NodeStatus::SUCCESS);
}





}  // namespace hri

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<dialog::GetGuestInfo>(
        name, "/kb/query", config);
    };

  factory.registerBuilder<dialog::GetGuestInfo>("GetGuestInfo", builder);
}

