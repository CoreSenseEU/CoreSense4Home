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

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "robocup_hri/check_policy.hpp"
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
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    image_topic_, 10, std::bind(&CheckPolicy::image_callback, this, _1));
}

void CheckPolicy::on_tick()
{
  rclcpp::spin_some(node_->get_node_base_interface());
  RCLCPP_DEBUG(node_->get_logger(), "CheckPolicy ticked");
  if (!image_) {
    RCLCPP_ERROR(node_->get_logger(), "No image received");
    setStatus(BT::NodeStatus::IDLE);
    return;
  }

  std::string text_;
  getInput("question", text_);

  std::string prompt_ = text_ + ". Please answer only with 'yes' or 'no'";
  goal_.prompt = prompt_;
  goal_.image = *image_;
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

void CheckPolicy::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  image_ = msg;
}

BT::NodeStatus CheckPolicy::on_success()
{
  fprintf(stderr, "%s\n", result_.result->response.text.c_str());

  if (result_.result->response.text.empty() || result_.result->response.text == "{}") {
    return BT::NodeStatus::FAILURE;
  }
  std::string answer = result_.result->response.text;
  setOutput("output_text", answer);

  answer.erase(
    std::remove_if(
      answer.begin(), answer.end(),
      [](unsigned char c) {return !std::isalnum(c);}), answer.end());
  std::transform(
    answer.begin(), answer.end(), answer.begin(),
    [](unsigned char c) {return std::tolower(c);});

  if (answer.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  if (answer.find("yes") != std::string::npos) {
    value_ = true;
  } else if (answer.find("no") != std::string::npos) {
    value_ = false;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Not a valid answer: %s", answer.c_str());
    return BT::NodeStatus::FAILURE;
  }
  setOutput("output", value_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace dialog
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<dialog::CheckPolicy>(name, "/llava/generate_response", config);
    };

  factory.registerBuilder<dialog::CheckPolicy>("CheckPolicy", builder);
}
