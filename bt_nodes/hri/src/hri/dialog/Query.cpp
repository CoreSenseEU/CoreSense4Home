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

#include "hri/dialog/Query.hpp"

#include <iostream>
#include <sstream>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

#include "hri/dialog/Query.hpp"
#include "llama_msgs/action/generate_response.hpp"
#include "std_msgs/msg/int8.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "llama_msgs/action/generate_response.hpp"

namespace dialog
{

using namespace std::chrono_literals;
using namespace std::placeholders;
using json = nlohmann::json;

Query::Query(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(
    xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  publisher_start_ = node_->create_publisher<std_msgs::msg::Int8>("dialog_action", 10);
  publisher_start_->on_activate();

  client_ = rclcpp_action::create_client<llama_msgs::action::GenerateResponse>(
    node_, "/llama/generate_response");
}

void Query::halt()
{
  RCLCPP_INFO(node_->get_logger(), "Query halted");
}

BT::NodeStatus Query::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "Query ticked");
  

  auto msg_dialog_action = std_msgs::msg::Int8();

  msg_dialog_action.data = 2;

  publisher_start_->publish(msg_dialog_action);

  if (status() == BT::NodeStatus::IDLE || !is_goal_sent_) {
    return on_idle();
  }

  if (text_.empty() || text_ == "{}" ) {
    return BT::NodeStatus::FAILURE;
  }

  json response = json::parse(text_);
  std::string value_ = response["intention"];
  

  if (value_.size() == 0 || isInvalid(value_)) {
    RCLCPP_ERROR(node_->get_logger(), "Not recognized intention");
    return BT::NodeStatus::FAILURE;
  }
  
  setOutput("intention_value", value_);

  return BT::NodeStatus::SUCCESS;

}

BT::NodeStatus Query::on_idle()
{

  auto goal = llama_msgs::action::GenerateResponse::Goal();

  std::string text;
  getInput("text", text);
  getInput("intention", intention_);

  std::string prompt_ = "Given the sentence \"" + text + "\", extract the " + intention_ +
    " from the sentence and return "
    "it with the following JSON format:\n" +
    "{\n\t\"intention\": \"word extracted in the sentence\"\n}";
  goal.prompt = prompt_;
  goal.reset = true;
  goal.sampling_config.temp = 0.0;
  goal.sampling_config.grammar =
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

  RCLCPP_INFO(node_->get_logger(), "Sending goal");

  auto future_goal_handle = client_->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(
      node_->get_node_base_interface(),
      future_goal_handle) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {

    RCLCPP_ERROR(node_->get_logger(), "send_goal failed");
    is_goal_sent_ = false;
    return BT::NodeStatus::RUNNING;
  }

  auto goal_handle = future_goal_handle.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return BT::NodeStatus::RUNNING;
  }

  // Wait for the server to be done with the goal
  auto result_future = client_->async_get_result(goal_handle);

  RCLCPP_INFO(node_->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "get result call failed :(");
    return BT::NodeStatus::RUNNING;
  }

  auto wrapped_result = result_future.get();

  if(wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected");
    return BT::NodeStatus::RUNNING;
  }

  is_goal_sent_ = true;
  text_ = wrapped_result.result->response.text; 

  return BT::NodeStatus::RUNNING;
}

bool Query::isInvalid(std::string str) {
  bool invalid = false;
  if (str.find("sentence") != std::string::npos) {
    invalid = true;
  }
  return invalid;
}

}  // namespace dialog
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {

  factory.registerNodeType<dialog::Query>("Query");
}
