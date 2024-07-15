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

#include "perception/filter_prev_detections.hpp"

#include <limits>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

FilterPrevDetections::FilterPrevDetections(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  config().blackboard->get("tf_buffer", tf_buffer_);
//   config().blackboard->get("tf_broadcaster", tf_broadcaster_);
  config().blackboard->get("tf_listener", tf_listener_);

  RCLCPP_INFO(node_->get_logger(), "FilterPrevDetections initialized");
}

void FilterPrevDetections::halt() {RCLCPP_INFO(node_->get_logger(), "FilterPrevDetections halted");}

inline double distance(
  const geometry_msgs::msg::TransformStamped & t1,
  const geometry_msgs::msg::TransformStamped & t2)
{
  return std::sqrt(
    std::pow(t1.transform.translation.x - t2.transform.translation.x, 2) +
    std::pow(t1.transform.translation.y - t2.transform.translation.y, 2));
}

BT::NodeStatus FilterPrevDetections::tick()
{
  getInput("prev_detections", prev_detections_);
  getInput("new_detections", new_detections_);
  getInput("margin", margin_);
  getInput("frame_id", frame_id_);

  std::list<geometry_msgs::msg::TransformStamped> new_transforms;

  for (auto & detection : new_detections_) {
    geometry_msgs::msg::TransformStamped entity_transform_now_msg;
    try {
      entity_transform_now_msg = tf_buffer_->lookupTransform(
        frame_id_, detection,
        tf2::TimePointZero);
      new_transforms.push_back(entity_transform_now_msg);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        node_->get_logger(), "Could not transform %s to %s: %s",
        detection.c_str(), frame_id_, ex.what());
      RCLCPP_INFO(node_->get_logger(), "Cannot transform");
    }
  }

  for (auto & new_transform : new_transforms) {
    bool close = false;

    for (auto & prev_transform : prev_detections_->items) {
      if (distance(new_transform, prev_transform) < margin_) {
        RCLCPP_INFO(
          node_->get_logger(), "Detection %s is close to %s, removing it",
          new_transform.child_frame_id.c_str(),
          prev_transform.child_frame_id.c_str());
        close = true;
        break;
        // } else if (new_transform.transform.translation.x < 0.50 && new_transform.transform.translation.y < 0.50
        //          && new_transform.transform.translation.x > 8.0 && new_transform.transform.translation.y > 7.23
        // ) {
        //     RCLCPP_INFO(
        //         node_->get_logger(), "Detection %s is out of the apartment, removing it", new_transform.child_frame_id.c_str());
        //     close = true;
        //     break;
      }
    }

    if (!close) {
      RCLCPP_INFO(
        node_->get_logger(), "Detection %s is not close to any previous detection",
        new_transform.child_frame_id.c_str());

      auto size = prev_detections_->items.size();
      std::string detection_type = new_transform.child_frame_id.substr(
        0, new_transform.child_frame_id.find(
          "_"));
      new_transform.child_frame_id = detection_type + "_f" + std::to_string(size);

      prev_detections_->items.push_back(new_transform);
    }
  }
  RCLCPP_INFO(
    node_->get_logger(), "[FilterPrevDetections] Total: %lu detections",
    prev_detections_->items.size());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::FilterPrevDetections>("FilterPrevDetections");
}
