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

#include "configuration/get_perception_model.hpp"

#include <string>
#include <utility>

namespace configuration
{

using namespace std::chrono_literals;
using namespace std::placeholders;

GetPerceptionModel::GetPerceptionModel(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: configuration::BtServiceNode<rcl_interfaces::srv::GetParameters,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
}

void GetPerceptionModel::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "GetPerceptionModelNode ticked");

  request_->names.push_back("model");
}

void GetPerceptionModel::on_result()
{
  if (result_.values.size() > 0) {
    std::cout << "Success GetPerceptionNode" << std::endl;
    setOutput("model_name", result_.values[0].string_value);

    std::cout << result_.values[0].string_value << std::endl;
    setStatus(BT::NodeStatus::SUCCESS);
    
  } else {
    std::cout << "Failure SetPerceptionNode" << std::endl;
    // setOutput("listen_text", result_.result->text);
    setStatus(BT::NodeStatus::FAILURE);
  }
}

}  // namespace perception

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<configuration::GetPerceptionModel>(
        name, "/yolo/yolov8_node/get_parameters", config);
    };

  factory.registerBuilder<configuration::GetPerceptionModel>("GetPerceptionModel", builder);
}
