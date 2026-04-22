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
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <regex>
#include <algorithm>
#include <cctype>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "hri/dialog/Query.hpp"
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

  std::string prompt_ = "";

  if(intention_.empty()){
    prompt_= "You are a robot named Tiago who is participating in the Robocup with the Gentlebots team from Spain, made up of the Rey Juan Carlos University of Madrid and the University of León. Answer the following question: \"" + text_ + "\"";
  }else{
    prompt_ = "Given the sentence \"" + text_ + "\", extract the " + intention_ +
    " from the sentence and return "
    "it with the following JSON format:\n" +
    "{\n\t\"intention\": \"word extracted in the sentence\"\n}";
  }
  goal_.prompt = prompt_;
  goal_.reset = true;
  goal_.sampling_config.temp = 0.0;
  // Fix 1: fixed-key grammar forces LLM to always emit {"intention": "..."}
  goal_.sampling_config.grammar =
    R"(root ::= "{" ws "\"intention\"" ws ":" ws "\"" chars "\"" ws "}"
chars ::= [^"]*
ws ::= ([ \t\n] ws)?)";

  auto msg_dialog_action = std_msgs::msg::Int8();

  msg_dialog_action.data = 2;

  publisher_start_->publish(msg_dialog_action);
}

std::string sanitize_llm_output(std::string text)
{
  // 1) quitar bloques <think>...</think>
  text = std::regex_replace(text, std::regex(R"(<think>.*?</think>)"), "");

  // 2) quitar tags sueltos <think>, </think> y cualquier otro <...>
  text = std::regex_replace(text, std::regex(R"(</?think>)"), "");
  text = std::regex_replace(text, std::regex(R"(<[^>]+>)"), "");

  // 3) si existe 'the guest is', quedarse desde ahí
  std::string anchor = "the guest is";
  auto pos = text.find(anchor);
  if (pos != std::string::npos) {
    text = text.substr(pos);
  }

  // 4) quitar saltos de línea
  text = std::regex_replace(text, std::regex(R"([\r\n\t]+)"), " ");

  // 5) quitar comillas y paréntesis
  text = std::regex_replace(text, std::regex(R"(["])"), "");
  text = std::regex_replace(text, std::regex(R"([()])"), "");
  text = std::regex_replace(text, std::regex(R"([!])"), "");

  // 6) cambiar guiones por espacio
  text = std::regex_replace(text, std::regex(R"(-)"), " ");

  // 6) cambiar comillas por espacio
  text = std::regex_replace(text, std::regex(R"(,)"), " ");

  // 7) colapsar espacios múltiples
  text = std::regex_replace(text, std::regex(R"(\s{2,})"), " ");

  // 9) quedarnos con una sola frase si hay varias
  // auto dot_pos = text.find('.');
  // if (dot_pos != std::string::npos) {
  //   text = text.substr(0, dot_pos + 1);
  // }

  return text;
}

BT::NodeStatus Query::on_success()
{
  fprintf(stderr, "%s\n", result_.result->response.text.c_str());

  if (result_.result->response.text.empty() || result_.result->response.text == "{}") {
    return BT::NodeStatus::FAILURE;
  }

  json response;
  try {
    response = json::parse(result_.result->response.text);
  } catch (json::parse_error & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse JSON response: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  // Try the expected key first
  std::string value_;
  auto it = response.find("intention");
  if (it != response.end() && it->is_string()) {
    value_ = it->get<std::string>();
  }

  value_ = sanitize_llm_output(value_);

  // Fix 2: fallback — scan all values for the first non-empty string
  if (value_.empty()) {
    for (auto & [key, val] : response.items()) {
      if (val.is_string() && !val.get<std::string>().empty()) {
        value_ = val.get<std::string>();
        RCLCPP_WARN(
          node_->get_logger(),
          "'intention' key not found, using value from key '%s'", key.c_str());
        break;
      }
    }
  }

  if (value_.empty()) {
    value_ = "unknown";
  }

  RCLCPP_INFO(node_->get_logger(), "Extracted intention: %s", value_.c_str());
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
