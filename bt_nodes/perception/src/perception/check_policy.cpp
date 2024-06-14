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

#include "perception/check_policy.hpp"

#include <string>
#include <utility>

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

CheckPolicy::CheckPolicy(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: perception::BtServiceNode<vqa_interfaces::srv::AnswerBooleanQuestion, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
}

void CheckPolicy::on_tick() 
{
  RCLCPP_DEBUG(node_->get_logger(), "CheckPolicy ticked");
  getInput("question", question_);
  request_->question = question_;  
}

void CheckPolicy::on_result() 
{
  if (result_.success ) {    
    setStatus(BT::NodeStatus::SUCCESS);
    setOutput("output", result_.answer);

  } else {
    std::string error_msg = result_.error_msg.data();
    std::cerr << "[CheckPolicy]Error: " << error_msg << std::endl;
    setStatus(BT::NodeStatus::FAILURE);
  }
}

}  // namespace perception

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<perception::CheckPolicy>(name, "answer_boolean_question", config);
    };

  factory.registerBuilder<perception::CheckPolicy>("CheckPolicy", builder);
}
