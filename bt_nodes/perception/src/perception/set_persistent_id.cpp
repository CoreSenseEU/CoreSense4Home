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

#include "perception/set_persistent_id.hpp"

#include <string>
#include <iostream>

namespace perception
{

using namespace std::chrono_literals;

SetPersistentId::SetPersistentId(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: perception::BtServiceNode<
    yolo_msgs::srv::SetPersitentID,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(xml_tag_name, action_name, conf)
{
}

void SetPersistentId::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "SetPersistentId ticked");

  int id;
  if (!getInput("id", id)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing ID in SetPersistentId");
  }

  request_->id = id;
}

void SetPersistentId::on_result()
{
  if (result_.success) {
    std::cout << "Success SetPersistentId" << std::endl;
    setStatus(BT::NodeStatus::SUCCESS);
  } else {
    std::cout << "Failure SetPersistentId: " << result_.message << std::endl;
    setStatus(BT::NodeStatus::FAILURE);
  }
}

}  // namespace perception

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<perception::SetPersistentId>(
        name, "set_persitent_id", config);
    };

  factory.registerBuilder<perception::SetPersistentId>("SetPersistentId", builder);
}
