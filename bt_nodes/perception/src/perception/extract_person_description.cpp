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

#include "perception/extract_person_description.hpp"

#include <limits>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "perception_system/PerceptionUtils.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

ExtractPersonDescription::ExtractPersonDescription(
  const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), gestures_({
    {"pointing_right", {3, 4}},
    {"pointing_left", {0, 1}},

    {"waving", {5, 6, 7}},
    {"raising", {6}}
  }),
  pose_names_({
    {0, "lying"},
    {1, "sitting"},
    {2, "standing"},
    {-1, "unknown"},
  })
{
  config().blackboard->get("node", node_);

  // pl::getInstance(node_)->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  // pl::getInstance(node_)->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}

BT::NodeStatus ExtractPersonDescription::tick()
{

  getInput("person_id", person_id_);

  RCLCPP_INFO(
    node_->get_logger(), "[ExtractPersonDescription] person_id is %s",
    person_id_.c_str());

  pl::getInstance(node_)->set_interest(person_id_, true);
  pl::getInstance(node_)->update(true);

  RCLCPP_INFO(
    node_->get_logger(), "[ExtractPersonDescription] Describe %s",
    person_id_.c_str());

  auto detections = pl::getInstance(node_)->get_by_type(person_id_);

  RCLCPP_INFO(
    node_->get_logger(), "[ExtractPersonDescription] detections size: %d",
    detections.size());

  if (detections.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[ExtractPersonDescription] No detections");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG(
    node_->get_logger(),
    "[ExtractPersonDescription] Processing detections...");

  getInput("interest", interest_);

  if (interest_ == "pose") {

    RCLCPP_INFO(
      node_->get_logger(),
      "[ExtractPersonDescription] Pose is number %d", detections[0].body_pose);
    if (detections[0].body_pose == -1) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[ExtractPersonDescription] Pose is unknown");
      return BT::NodeStatus::FAILURE;
    }

    description_ = pose_names_[detections[0].body_pose];
    RCLCPP_INFO(
      node_->get_logger(),
      "[ExtractPersonDescription] The pose of the person is: %s",
      description_.c_str());

    setOutput("description", description_);

    return BT::NodeStatus::SUCCESS;
  }

  if (interest_ == "gesture") {
    int pointing_direction = detections[0].pointing_direction;
    RCLCPP_INFO(
      node_->get_logger(),
      "[ExtractPersonDescription] The pointing_direction is: %d", pointing_direction);

    for (const auto & gesture : gestures_) {
      const auto & key = gesture.first;
      const auto & direction = gesture.second;

      if (std::find(direction.begin(), direction.end(), pointing_direction) !=
        direction.end())
      {
        description_ = key;
        setOutput("description", description_);
        RCLCPP_INFO(
          node_->get_logger(),
          "[ExtractPersonDescription] The gesture of the person is: %s",
          description_.c_str());
        return BT::NodeStatus::SUCCESS;
      }

    }

    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}
void ExtractPersonDescription::halt()
{
  RCLCPP_DEBUG(node_->get_logger(), "ExtractPersonDescription halted");
}

} // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::ExtractPersonDescription>(
    "ExtractPersonDescription");
}
