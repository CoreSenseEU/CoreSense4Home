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

#include "hri/dialog/store_guest_info.hpp"
#include <regex>

using std::placeholders::_1;
using namespace std::chrono_literals;


namespace dialog
{

StoreGuestInfo::StoreGuestInfo(
  const std::string & xml_tag_name, const std::string & srv_name,
  const BT::NodeConfiguration & conf)
: hri::BtServiceNode<
    kb_msgs::srv::Query,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(xml_tag_name, srv_name, conf)
{
  config().blackboard->get("node", node_);
  this->kb_publisher_ = node_->create_publisher<std_msgs::msg::String>("/kb/add_fact", 10);
}

void StoreGuestInfo::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "[StoreGuestInfo] ticked");
  rclcpp::spin_some(node_->get_node_base_interface());

  getInput("guest_name", guest_name_);
  getInput("guest_drink", guest_drink_);
  getInput("guest_description", guest_description_);
  // Strip characters that cause Turtle/OWL syntax errors:
  // - dots (.): break Turtle statement terminator
  // - backslashes: LLM sometimes outputs \, or \\ in descriptions; neither is valid here
  auto strip_chars = [](const std::string & in, const std::string & chars) {
    std::string out;
    out.reserve(in.size());
    for (char c : in) {
      if (chars.find(c) == std::string::npos) {
        out += c;
      }
    }
    return out;
  };
  guest_description_ = strip_chars(guest_description_, ".\\/");

  if (guest_name_.empty() || guest_drink_.empty()) {
    setStatus(BT::NodeStatus::FAILURE);
  }


  request_->patterns.push_back("?guest rdf:type oro:Person");
  request_->vars.push_back("?guest");

  setStatus(BT::NodeStatus::SUCCESS);
}

std::string obtain_guest_id(const std::string & json)
{
  std::regex guest_regex("guest([0-9]+)");
  std::smatch match;

  int max_id = 0;

  auto begin = json.cbegin();
  auto end = json.cend();

  while (std::regex_search(begin, end, match, guest_regex)) {
    int id = std::stoi(match[1]);
    max_id = std::max(max_id, id);
    begin = match.suffix().first;
  }

  return "guest" + std::to_string(max_id + 1);
}

static std::string sanitize_turtle_literal(const std::string & s)
{
  // Strip ALL backslashes unconditionally — LLM-generated descriptions never need
  // legitimate escape sequences, and the KB parser (OWL/Turtle) chokes on anything
  // that isn't one of the five valid escapes (\t \n \r \\ \").
  // Also strip dots (.) which break Turtle statement terminators.
  std::string result;
  result.reserve(s.size());

  for (size_t i = 0; i < s.size(); ++i) {
    char c = s[i];
    if (c == '\\') {
      // Drop ALL backslashes unconditionally — skip the backslash.
      // If there is a following character, keep it (it's the intended content).
      if (i + 1 < s.size()) {
        ++i;
        result += s[i];
      }
      // Trailing backslash at end of string: just drop it
    } else if (c == '"') {
      result += "\\\"";   // escape bare double quotes for Turtle
    } else if (c == '\n' || c == '\r' || c == '-' ) {
      result += ' ';
    } else {
      result += c;
    }
  }
  return result;
}

void StoreGuestInfo::on_result()
{
  RCLCPP_DEBUG(node_->get_logger(), "[StoreGuestInfo] result received");

  if (!result_.error_msg.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[StoreGuestInfo] error");
    setStatus(BT::NodeStatus::FAILURE);
  }

  std::string guest_id = obtain_guest_id(result_.json);

  std_msgs::msg::String fact_msg;

  // Add guest
  fact_msg.data = guest_id + " rdf:type oro:Person";
  kb_publisher_->publish(fact_msg);

  // Add guest name
  fact_msg.data = guest_id + " oro:hasName \"" + guest_name_ + "\"";
  kb_publisher_->publish(fact_msg);

  //Add guest drink
  fact_msg.data = guest_id + " oro:hasFavoriteDrink \"" + guest_drink_ + "\"";
  kb_publisher_->publish(fact_msg);

  if (!guest_description_.empty()) {
    guest_description_ = sanitize_turtle_literal(guest_description_);
    fact_msg.data = guest_id + " oro:description \"" + guest_description_ + "\"";
    kb_publisher_->publish(fact_msg);
  }

  // Add robot attending guest
  fact_msg.data = "robot1 oro:attends " + guest_id;
  kb_publisher_->publish(fact_msg);

  setStatus(BT::NodeStatus::SUCCESS);
}


}  // namespace hri

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<dialog::StoreGuestInfo>(
      name, "/kb/query", config);
  };

  factory.registerBuilder<dialog::StoreGuestInfo>("StoreGuestInfo", builder);
}
