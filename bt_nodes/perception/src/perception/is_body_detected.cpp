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

#include "perception/is_body_detected.hpp"

#include <limits>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "sensor_msgs/msg/image.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsBodyDetected::IsBodyDetected(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  max_depth_(std::numeric_limits<double>::max()),
  max_entities_(1)
{
  config().blackboard->get("node", node_);
  ids_list_sub_ = node_->create_subscription<hri_msgs::msg::IdsList>(
    "/humans/bodies/tracked", 10,
    [this](const hri_msgs::msg::IdsList::SharedPtr msg) {
      RCLCPP_INFO_ONCE(node_->get_logger(), "IDs list received");
      last_ids_list_msg_ = msg;
    });
}

BT::NodeStatus IsBodyDetected::tick()
{
  rclcpp::spin_some(node_->get_node_base_interface());
  if (!last_ids_list_msg_) {
    RCLCPP_ERROR(node_->get_logger(), "[IsBodyDetected] No bodies detected so far");
    return BT::NodeStatus::FAILURE;
  }
  if (last_ids_list_msg_->ids.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[IsBodyDetected] No bodies detected");
    return BT::NodeStatus::FAILURE;
  }
  setOutput("best_detection", "waist_"+last_ids_list_msg_->ids[0]);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsBodyDetected>("IsBodyDetected");
}
