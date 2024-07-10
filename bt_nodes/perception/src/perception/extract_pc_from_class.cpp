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

#include "perception/extract_pc_from_class.hpp"

#include <string>
#include <utility>

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

ExtractPcFromClass::ExtractPcFromClass(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: perception::BtServiceNode<
    perception_system_interfaces::srv::IsolatePCClasses,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(xml_tag_name, action_name, conf)
{
}

void ExtractPcFromClass::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "ExtractPcFromClass ticked");

  std::string selected_object;
  getInput("selected_object", selected_object);
  std::cout << "ExtractPcFromClass selected_object: " << selected_object << std::endl;

  request_->classes = {selected_object};
}

void ExtractPcFromClass::on_result()
{
  if (result_.success) {
    std::cout << "Success" << std::endl;
    setStatus(BT::NodeStatus::SUCCESS);
    setOutput("class_pc", result_.filtered_pc);
  } else {
    std::cout << "Failure" << std::endl;
    setStatus(BT::NodeStatus::FAILURE);
  }
}

}  // namespace perception

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<perception::ExtractPcFromClass>(
        name, "perception_system/isolate_pc_classes", config);
    };

  factory.registerBuilder<perception::ExtractPcFromClass>("ExtractPcFromClass", builder);
}
