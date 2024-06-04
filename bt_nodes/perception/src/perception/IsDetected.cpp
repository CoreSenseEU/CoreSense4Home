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

#include "perception/IsDetected.hpp"

#include <limits>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "perception_system/PerceptionUtils.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsDetected::IsDetected(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  max_depth_(std::numeric_limits<double>::max()),
  max_entities_(1),
  colors_({
    {"blue", cv::Scalar(120,255,255)}, {"yellow", cv::Scalar(30,255, 255)}, {"black", cv::Scalar(0,0,0)},
    {"white", cv::Scalar(0, 0, 255)}, {"red", cv::Scalar(0,255,255)}, {"orange", cv::Scalar(30, 255, 255)},
    {"gray", cv::Scalar(0, 128, 128)}})
{
  config().blackboard->get("node", node_);

  // node_->add_activation("perception_system/perception_people_detection")

  getInput("interest", interest_);
  getInput("cam_frame", cam_frame_);
  getInput("confidence", threshold_);
  getInput("max_entities", max_entities_);
  getInput("order", order_);
  getInput("max_depth", max_depth_);
  getInput("person_id", person_id_);
}

BT::NodeStatus IsDetected::tick()
{
  rclcpp::spin_some(node_->get_node_base_interface());
  getInput("person_id", person_id_);
  getInput("color", color_);

  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_INFO(node_->get_logger(), "IsDetected ticked");
    config().blackboard->get("tf_buffer", tf_buffer_);
    // config().blackboard->get("tf_broadcaster", tf_broadcaster_);
  }

  RCLCPP_DEBUG(node_->get_logger(), "IsDetected ticked");
  pl::getInstance(node_)->set_interest(interest_, true);
  pl::getInstance(node_)->update(35);

  auto detections = pl::getInstance(node_)->get_by_type(interest_);

  if (detections.empty()) {
    // RCLCPP_WARNING(node_->get_logger(), "[IsDetected] No detections");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG(node_->get_logger(), "[IsDetected] Processing %d detections...", detections.size());

  if (order_ == "color") {
    // sorted by the distance to the color person we should sort it by distance and also by left to right or right to left
    RCLCPP_DEBUG(node_->get_logger(), "[IsDetected] Sorting detections by color");
    std::sort(
      detections.begin(), detections.end(), [this](const auto & a, const auto & b) {
        return perception_system::diffIDs(this->person_id_, a.color_person) <
        perception_system::diffIDs(this->person_id_, b.color_person);
      });
  } else if (order_ == "depth") {
    RCLCPP_DEBUG(node_->get_logger(), "[IsDetected] Sorting detections by depth");
    std::sort(
      detections.begin(), detections.end(), [this](const auto & a, const auto & b) {
        return a.center3d.position.z < b.center3d.position.z;
      });
  }
  // auto pub = node_->create_publisher<sensor_msgs::msg::Image>(
  //   "/object_detected", 10);

  // pub->publish(detections[0].image);

  setOutput("best_detection", detections[0].class_name);
  RCLCPP_INFO(node_->get_logger(), "[IsDetected] Detections sorted");
  // implement more sorting methods

  RCLCPP_DEBUG(node_->get_logger(), "[IsDetected] Max Depth: %f", max_depth_);
  RCLCPP_DEBUG(node_->get_logger(), "[IsDetected] Threshold: %f", threshold_);
  auto entity_counter = 0;
  for (auto it = detections.begin(); it != detections.end() && entity_counter < max_entities_; ) {
    auto const & detection = *it;

    auto const detection_id_colors = perception_system::getHSVFromUniqueID(detection.color_person);

    if (detection.score <= threshold_ || 
        detection.center3d.position.z > max_depth_ ||
        (color_ != ""  &&
         std::abs(detection_id_colors[0][0] - colors_[color_][0]) > hue_threshold_ &&
         std::abs(detection_id_colors[0][1] - colors_[color_][1]) > saturation_threshold_ &&
         std::abs(detection_id_colors[0][2] - colors_[color_][2]) > value_threshold_))
    {
      RCLCPP_DEBUG(
        node_->get_logger(), "[IsDetected] Removing detection %s", detection.class_name.c_str());
      RCLCPP_DEBUG(node_->get_logger(), "[IsDetected] Score: %f", detection.score);
      RCLCPP_DEBUG(node_->get_logger(), "[IsDetected] Depth: %f", detection.center3d.position.z);
      it = detections.erase(it);

    } else {
      frames_.push_back(detection.class_name + "_" + std::to_string(entity_counter));
      if (pl::getInstance(node_)->publicTF(detection, std::to_string(entity_counter)) == -1) {
        return BT::NodeStatus::FAILURE;
      }
      ++it;
      ++entity_counter;
    }
  }

  RCLCPP_DEBUG(node_->get_logger(), "[IsDetected] Detections sorted and filtered");
  if (frames_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[IsDetected] No detections after filter");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("frames", frames_);
  frames_.clear();
  // print pointing_direction
  RCLCPP_INFO(node_->get_logger(), "Pointing direction: %d", detections[0].pointing_direction);

  RCLCPP_DEBUG(node_->get_logger(), "[IsDetected] Detections published");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsDetected>("IsDetected");
}
