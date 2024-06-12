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

#include "perception/is_gesturing.hpp"
#include <map>

#include <limits>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "perception_system/PerceptionUtils.hpp"

namespace perception {

using namespace std::chrono_literals;
using namespace std::placeholders;

using pl = perception_system::PerceptionListener;

IsGesturing::IsGesturing(const std::string &xml_tag_name,
                         const BT::NodeConfiguration &conf)
    : BT::ConditionNode(xml_tag_name, conf) {
  config().blackboard->get("node", node_);

  getInput("type", type_);
  gestures_["waving"] = {5, 6, 7};
  gestures_["raising"] = {6};
  gestures_["pointing"] = {0, 1, 3, 4};
}

BT::NodeStatus IsGesturing::tick() {
  if (type_.empty() || type_ == "none") {
    RCLCPP_INFO(node_->get_logger(), "No gesture specified");
    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_INFO(node_->get_logger(), "IsGesturing ticked");

  if (gestures_.find(type_) != gestures_.end()) {
    RCLCPP_INFO(node_->get_logger(), "Unknown gesture");
    return BT::NodeStatus::FAILURE;
  }

  pl::getInstance(node_)->set_interest("person", true);
  pl::getInstance(node_)->update(true);

  std::vector<perception_system_interfaces::msg::Detection> detections;
  detections = pl::getInstance(node_)->get_by_type("person");

  if (detections.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "No person detections");
    return BT::NodeStatus::FAILURE;
  }

  perception_system_interfaces::msg::Detection best_detection;
  auto it = std::find_if(detections.begin(), detections.end(),
                         [this](const auto &obj) {
                           return std::find(gestures_[this->type_].begin(),
                                            gestures_[this->type_].end(),
                                            obj.pointing_direction) !=
                                  gestures_[this->type_].end();
                         });
  if (it != detections.end()) {
    setOutput("person_id", it->color_person);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

} // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsGesturing>("IsGesturing");
}
