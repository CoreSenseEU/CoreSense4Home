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

#include "robocup_hri/dialog/Query.hpp"

#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "robocup_hri/dialog/Query.hpp"
#include "llama_msgs/action/generate_response.hpp"
#include "std_msgs/msg/int8.hpp"

namespace dialog
{

using namespace std::chrono_literals;
using namespace std::placeholders;
using json = nlohmann::json;

Query::Query(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: dialog::BtActionNode<
    llama_msgs::action::GenerateResponse, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
  publisher_start_ = node_->create_publisher<std_msgs::msg::Int8>("dialog_action", 10);
}

void Query::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "Query ticked");
  std::string text_;
  getInput("text", text_);
  getInput("intention", intention_);
  std::string prompt_ = "Given the sentence \"" + text_ + "\", extract the " + intention_ +
    " from the sentence and return "
    "it with the following JSON format:\n" +
    "{\n\t\"intention\": \"word extracted in the sentence\"\n}";
  goal_.prompt = prompt_;
  goal_.reset = true;
  goal_.sampling_config.temp = 0.0;
  goal_.sampling_config.grammar =
    R"(root   ::= object
value  ::= object | array | string | number | ("true" | "false" | "null") ws

object ::=
  "{" ws (
            string ":" ws value
    ("," ws string ":" ws value)*
  )? "}" ws

array  ::=
  "[" ws (
            value
    ("," ws value)*
  )? "]" ws

string ::=
  "\"" (
    [^"\\] |
    "\\" (["\\/bfnrt] | "u" [0-9a-fA-F] [0-9a-fA-F] [0-9a-fA-F] [0-9a-fA-F]) # escapes
  )* "\"" ws

number ::= ("-"? ([0-9] | [1-9] [0-9]*)) ("." [0-9]+)? ([eE] [-+]? [0-9]+)? ws

# Optional space: by convention, applied in this grammar after literal chars when allowed
ws ::= ([ \t\n] ws)?)";

  auto msg_dialog_action = std_msgs::msg::Int8();

  msg_dialog_action.data = 2;

  publisher_start_->publish(msg_dialog_action);
}

BT::NodeStatus Query::on_success()
{
  fprintf(stderr, "%s\n", result_.result->response.text.c_str());

  if (result_.result->response.text.empty() || result_.result->response.text == "{}") {
    return BT::NodeStatus::FAILURE;
  }

  json response = json::parse(result_.result->response.text);
  std::string value_ = response["intention"];
  fprintf(stderr, "%s\n", value_.c_str());

  if (value_.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("intention_value", value_);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace dialog
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<dialog::Query>(name, "/llama/generate_response", config);
    };

  factory.registerBuilder<dialog::Query>("Query", builder);
}
