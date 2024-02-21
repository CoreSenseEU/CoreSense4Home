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

#include "hri/GenerateTextNoSpeech.hpp"
#include <algorithm>

using std::placeholders::_1;
using namespace std::chrono_literals; 

namespace hri
{

GenerateTextNoSpeech::GenerateTextNoSpeech(const std::string & xml_tag_name,
                                                 const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);  
}

void
GenerateTextNoSpeech::halt()
{
  RCLCPP_INFO(node_->get_logger(), "GenerateTextNoSpeech halted");
}


BT::NodeStatus
GenerateTextNoSpeech::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "GenerateTextFromObjects ticked");
  rclcpp::spin_some(node_);
  std::vector<moveit_msgs::msg::CollisionObject::SharedPtr> detected_objects = {};
  getInput("detected_objects",detected_objects);

  if (detected_objects.empty()) {
    RCLCPP_INFO(node_->get_logger(), "No objects detected");
    return BT::NodeStatus::FAILURE;
  }
  if(selected_object_ >= detected_objects.size())
  {
    selected_object_ = 0;
    return BT::NodeStatus::FAILURE;
  }
  // Add the available objects in detected_objects to the string :
  
  std::string question = "Please leave only the object you want me to pick and remove the rest";
  
  setOutput("output_text", question);

  RCLCPP_INFO(node_->get_logger(), "Selected object: %s", detected_objects[selected_object_]->id.c_str());
  setOutput("selected_object", detected_objects[selected_object_]);

  selected_object_++;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hri::GenerateTextNoSpeech>("GenerateTextNoSpeech");
}
