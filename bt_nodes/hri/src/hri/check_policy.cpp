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


#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <regex>
#include <algorithm>
#include <cctype>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "hri/check_policy.hpp"
#include "llama_msgs/action/generate_response.hpp"
#include "std_msgs/msg/int8.hpp"


namespace dialog
{

using namespace std::chrono_literals;
using namespace std::placeholders;
using json = nlohmann::json;

CheckPolicy::CheckPolicy(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: dialog::BtActionNode<
    llama_msgs::action::GenerateResponse, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
  getInput("image_topic", image_topic_);
  image_sub_ = node_->create_subscription<perception_system_interfaces::msg::DetectionArray>(
    image_topic_, 10, std::bind(&CheckPolicy::image_callback, this, _1));
}

void CheckPolicy::on_tick()
{
  rclcpp::spin_some(node_->get_node_base_interface());
  RCLCPP_DEBUG(node_->get_logger(), "CheckPolicy ticked");
  RCLCPP_INFO(node_->get_logger(), "CheckPolicy ticked");
  if (!image_) {
    RCLCPP_ERROR(node_->get_logger(), "No image received yet");
    goal_.prompt.clear();
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Image received, proceeding with CheckPolicy");

  std::string text_;
  getInput("question", text_);

  std::string prompt_ = text_;
  goal_.prompt = prompt_;
  goal_.images.clear();
  goal_.images.push_back(*image_);
  goal_.reset = true;
  goal_.sampling_config.temp = 0.0;
//   goal_.sampling_config.grammar =
//     R"(root   ::= object
// value  ::= object | array | string | number | ("true" | "false" | "null") ws

// object ::=
//   "{" ws (
//             string ":" ws value
//     ("," ws string ":" ws value)*
//   )? "}" ws

// array  ::=
//   "[" ws (
//             value
//     ("," ws value)*
//   )? "]" ws

// string ::=
//   "\"" (
//     [^"\\] |
//     "\\" (["\\/bfnrt] | "u" [0-9a-fA-F] [0-9a-fA-F] [0-9a-fA-F] [0-9a-fA-F]) # escapes
//   )* "\"" ws

// number ::= ("-"? ([0-9] | [1-9] [0-9]*)) ("." [0-9]+)? ([eE] [-+]? [0-9]+)? ws

// # Optional space: by convention, applied in this grammar after literal chars when allowed
// ws ::= ([ \t\n] ws)?)";
}

void CheckPolicy::image_callback(
  const perception_system_interfaces::msg::DetectionArray::SharedPtr msg)
{
  image_ = std::make_shared<sensor_msgs::msg::Image>(msg->source_img);
  RCLCPP_INFO_ONCE(node_->get_logger(), "Image received in CheckPolicy");
}

std::string trim_copy(const std::string & s)
{
  auto start = std::find_if_not(s.begin(), s.end(),
    [](unsigned char c) { return std::isspace(c); });
  auto end = std::find_if_not(s.rbegin(), s.rend(),
    [](unsigned char c) { return std::isspace(c); }).base();

  if (start >= end) {
    return "";
  }
  return std::string(start, end);
}

std::string sanitize_llm_output(std::string text)
{
  // 1) quitar bloques <think>...</think>
  text = std::regex_replace(text, std::regex(R"(<think>.*?</think>)"), "");

  // 2) quitar tags sueltos <think>, </think> y cualquier otro <...>
  text = std::regex_replace(text, std::regex(R"(</?think>)"), "");
  text = std::regex_replace(text, std::regex(R"(<[^>]+>)"), "");

  // 3) si existe 'the guest is', quedarse desde ahí
  // std::string anchor = "the guest is";
  // auto pos = text.find(anchor);
  // if (pos != std::string::npos) {
  //   text = text.substr(pos);
  // }

  // 4) quitar saltos de línea
  text = std::regex_replace(text, std::regex(R"([\r\n\t]+)"), " ");

  // 5) quitar comillas y paréntesis
  text = std::regex_replace(text, std::regex(R"(["])"), "");
  text = std::regex_replace(text, std::regex(R"([()])"), "");
  text = std::regex_replace(text, std::regex(R"([!])"), "");

  // 6) cambiar guiones por espacio
  text = std::regex_replace(text, std::regex(R"(-)"), " ");

  // 7) colapsar espacios múltiples
  text = std::regex_replace(text, std::regex(R"(\s{2,})"), " ");

  // 8) trim
  text = trim_copy(text);

  // 9) quedarnos con una sola frase si hay varias
  // auto dot_pos = text.find('.');
  // if (dot_pos != std::string::npos) {
  //   text = text.substr(0, dot_pos + 1);
  // }

  return text;
}

BT::NodeStatus CheckPolicy::on_success()
{
  fprintf(stderr, "%s\n", result_.result->response.text.c_str());
  RCLCPP_INFO(node_->get_logger(), "CheckPolicy succeeded");
  RCLCPP_INFO(
    node_->get_logger(), "LLM response: %s",
    result_.result->response.text.c_str());

  if (result_.result->response.text.empty() || result_.result->response.text == "{}") {
    return BT::NodeStatus::FAILURE;
  }

  std::string answer = sanitize_llm_output(result_.result->response.text);

  RCLCPP_INFO(
    node_->get_logger(), "CheckPolicy extracted answer: %s",
    answer.c_str());

  if (answer.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("output_text", answer);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace dialog
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<dialog::CheckPolicy>(name, "/llama/generate_response", config);
  };

  factory.registerBuilder<dialog::CheckPolicy>("CheckPolicy", builder);
}
