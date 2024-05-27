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

#include "perception/clear_octomap.hpp"

#include <string>
#include <utility>

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

ClearOctomap::ClearOctomap(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: perception::BtServiceNode<std_srvs::srv::Empty,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
} 

void ClearOctomap::on_tick()
{
  RCLCPP_INFO(node_->get_logger(), "ClearOctomap ticked");
}

void ClearOctomap::on_result()
{

  setStatus(BT::NodeStatus::SUCCESS);
    
}

}  // namespace perception

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<perception::ClearOctomap>(
        name, "clear_octomap", config);
    };

  factory.registerBuilder<perception::ClearOctomap>("ClearOctomap", builder);
}
