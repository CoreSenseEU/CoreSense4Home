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

#include "configuration/get_model_path.hpp"

#include <filesystem>
#include <iostream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace configuration
{

GetModelPath::GetModelPath(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
}

BT::NodeStatus GetModelPath::tick()
{
  std::string model;
  if (!getInput("model", model)) {
    RCLCPP_ERROR(rclcpp::get_logger("GetModelPath"), "Missing required input [model]");
    return BT::NodeStatus::FAILURE;
  }

  try {
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("robocup_bringup");
    std::string model_path = pkg_share_dir + "/models/" + model;

    if (std::filesystem::exists(model_path)) {
      setOutput("model_path", model_path);
      RCLCPP_INFO(rclcpp::get_logger("GetModelPath"), "Found model at: %s", model_path.c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("GetModelPath"), "Model not found at: %s",
        model_path.c_str());
      return BT::NodeStatus::FAILURE;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GetModelPath"), "Failed to get package share directory: %s",
      e.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace configuration

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<configuration::GetModelPath>("GetModelPath");
}
