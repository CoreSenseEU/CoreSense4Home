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

#include "perception/extract_entity_color.hpp"

#include <limits>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "perception_system/PerceptionUtils.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

ExtractEntityColor::ExtractEntityColor(
  const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  getInput("interest", interest_);
  getInput("confidence", threshold_);

  // pl::getInstance(node_)->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  // pl::getInstance(node_)->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}

BT::NodeStatus ExtractEntityColor::tick()
{
  pl::getInstance(node_)->set_interest(interest_, true);
  pl::getInstance(node_)->update(30);

  RCLCPP_INFO(node_->get_logger(), "[ExtractEntityColor] Interest in %s", interest_.c_str());
  auto detections = pl::getInstance(node_)->get_by_type(interest_);

  if (detections.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[ExtractEntityColor] No detections");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG(node_->get_logger(), "[ExtractEntityColor] Processing detections...");

  std::sort(
    detections.begin(), detections.end(), [this](const auto & a, const auto & b) {
      return a.center3d.position.z < b.center3d.position.z;
    });

  if (detections[0].score < threshold_) {
    RCLCPP_ERROR(
      node_->get_logger(), "[ExtractEntityColor] Detection confidence is below threshold");
    return BT::NodeStatus::FAILURE;
  }
  setOutput("person_id", detections[0].color_person);
  RCLCPP_DEBUG(
    node_->get_logger(), "[ExtractEntityColor] Person color: %ld", detections[0].color_person);
  return BT::NodeStatus::SUCCESS;
}
void ExtractEntityColor::halt() {RCLCPP_DEBUG(node_->get_logger(), "ExtractEntityColor halted");}

}  // namespace perception

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<perception::ExtractEntityColor>("ExtractEntityColor");
}
