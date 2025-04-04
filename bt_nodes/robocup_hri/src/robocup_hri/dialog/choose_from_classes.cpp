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

#include "robocup_hri/dialog/choose_from_classes.hpp"

#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "llama_msgs/action/generate_response.hpp"
#include "std_msgs/msg/int8.hpp"

namespace dialog
{

using namespace std::chrono_literals;
using namespace std::placeholders;
using json = nlohmann::json;

ChooseFromClasses::ChooseFromClasses(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: dialog::BtActionNode<
    llama_msgs::action::GenerateResponse, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
}

void ChooseFromClasses::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "ChooseFromClasses ticked");
  getInput("class_options", class_options_);
  getInput("target_class", target_class_);

  auto classes_string = vector_to_string(class_options_);

  std::string prompt_ = "From the following list of object classes: " + classes_string +
    ", please select the one that is more related to the object class " +
    remove_suffix(target_class_, "_") +
    ". And return it with the following JSON format:\n" +
    "{\n\t\"answer\": \"object_class\"\n}";

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
}

BT::NodeStatus ChooseFromClasses::on_success()
{
  fprintf(stderr, "%s\n", result_.result->response.text.c_str());

  if (result_.result->response.text.empty() || result_.result->response.text == "{}") {
    return BT::NodeStatus::FAILURE;
  }

  json response = json::parse(result_.result->response.text);

  std::string value_ = response["answer"];

  if (value_.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "[ChooseFromClasses] Llama plane answer: %s", value_.c_str());

  RCLCPP_INFO(
    node_->get_logger(), "[ChooseFromClasses] Llama answer after removing suffix: %s",
    value_.c_str());

  std::string selected_class = retrieve_class(value_, class_options_);

  if (selected_class.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[ChooseFromClasses] Selected class is empty");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("selected_class", retrieve_class(value_, class_options_));
  value_ = value_.substr(0, value_.find("_"));
  setOutput("selected_class_text", value_);

  return BT::NodeStatus::SUCCESS;
}

std::string ChooseFromClasses::vector_to_string(const std::vector<std::string> & vec)
{
  return std::accumulate(
    vec.begin(), vec.end(), std::string("{"),
    [&](const std::string & acc, const std::string & s) {
      std::string modified_s = remove_suffix(s, "_");
      return acc + (acc == "{" ? "" : ", ") + modified_s;
    }) +
         "}";
}

std::string ChooseFromClasses::remove_suffix(const std::string & str, const std::string & suffix)
{
  if (
    str.size() >= suffix.size() &&
    str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0)
  {
    return str.substr(0, str.size() - suffix.size());
  }
  return str;
}

std::string ChooseFromClasses::retrieve_class(
  const std::string & text, const std::vector<std::string> & class_options)
{
  RCLCPP_INFO(node_->get_logger(), "[ChooseFromClasses] Retrieving from: %s in :", text.c_str());
  for (const auto & class_option : class_options) {
    RCLCPP_INFO(node_->get_logger(), "[ChooseFromClasses] %s", class_option.c_str());
    if (
      text.substr(0, text.find("_")).find(class_option.substr(0, class_option.find("_"))) !=
      std::string::npos)
    {
      return class_option;
    } else {
      return "";
    }
  }
}

}  // namespace dialog

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<dialog::ChooseFromClasses>(name, "/llama/generate_response", config);
    };

  factory.registerBuilder<dialog::ChooseFromClasses>("ChooseFromClasses", builder);
}
