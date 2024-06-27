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
     {"lower_blue", cv::Scalar(90,50,50)}, {"upper_blue", cv::Scalar(125,255,255)},
    {"lower_yellow", cv::Scalar(25,150,200)}, {"upper_yellow", cv::Scalar(30,255,255)},
    {"lower_black", cv::Scalar(0,0,0)}, {"upper_black", cv::Scalar(255,255,50)},
    {"lower_white", cv::Scalar(0,0,200)}, {"upper_white", cv::Scalar(255,30,255)},
    {"lower_red", cv::Scalar(170,150,180)}, {"upper_red", cv::Scalar(205,255,255)},
    {"lower_orange", cv::Scalar(10,120,120)}, {"upper_orange", cv::Scalar(20,255,255)},
    {"lower_gray", cv::Scalar(0,0,100)}, {"upper_gray", cv::Scalar(180,30,200)}}),
  gestures_({
      {"pointing_right", {0, 1}},
      {"pointing_left", {3, 4}},

      {"waving", {5, 6, 7}},
      {"rising_left", {6}},
      {"rising_left", {6}},
    }),
  pose_names_({
    {0, "lying"},
    {1, "sitting"},
    {2, "standing"},
    {-1, "unknown"},
  })
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
  getInput("gesture", gesture_);
  getInput("pose", pose_);

  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_INFO(node_->get_logger(), "IsDetected idle");
    config().blackboard->get("tf_buffer", tf_buffer_);
    // config().blackboard->get("tf_broadcaster", tf_broadcaster_);
  }

  RCLCPP_DEBUG(node_->get_logger(), "IsDetected ticked");
  pl::getInstance(node_)->set_interest(interest_, true);
  pl::getInstance(node_)->update(35);

  auto detections = pl::getInstance(node_)->get_by_type(interest_);

  if (detections.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[IsDetected] No detections");
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
    bool removed = false;

    if (detection.score > threshold_ && detection.center3d.position.z < max_depth_) {
      // Color filtering
      if (color_ != "unknown") {
        auto const detection_id_colors = perception_system::getHSVFromUniqueID(detection.color_person);
        std::string lower_color = "lower_" + color_;
        std::string upper_color = "upper_" + color_;

        cv::Scalar lower_bound = colors_[lower_color];
        cv::Scalar upper_bound = colors_[upper_color];

        cv::Scalar detection_color = detection_id_colors[0];

        double hue = detection_color[0];

        if (color_ == "red") {
          hue += 180;
        }

        RCLCPP_INFO(node_->get_logger(), "[IsDetected] Detection %s is %f %f %f", detection.unique_id.c_str(), detection_color[0], detection_color[1], detection_color[2]);

        if (hue >= lower_bound[0] && hue <= upper_bound[0] &&
            detection_color[1] >= lower_bound[1] && detection_color[1] <= upper_bound[1] &&
            detection_color[2] >= lower_bound[2] && detection_color[2] <= upper_bound[2]) {
          RCLCPP_DEBUG(node_->get_logger(), "[IsDetected] Detection %s is %s", detection.unique_id.c_str(), color_.c_str());
        } else {
          RCLCPP_DEBUG(node_->get_logger(), "[CountPeople] Detection %s is not %s", detection.unique_id.c_str(), color_.c_str());
          it = detections.erase(it);
          removed = true;
        }
      }

      // gesture filtering
      if (gesture_ != "unknown" && !removed) {
        if (std::find(gestures_[gesture_].begin(), gestures_[gesture_].end(), detection.pointing_direction) != gestures_[gesture_].end()) {
          RCLCPP_DEBUG(node_->get_logger(), "[IsDetected] Detection %s is %s", detection.unique_id.c_str(), gesture_.c_str());
        } else {
          RCLCPP_DEBUG(node_->get_logger(), "[IsDetected] Detection %s is not %s", detection.unique_id.c_str(), gesture_.c_str());
          it = detections.erase(it);
          removed = true;
        }
      }

      // pose filtering
      if (pose_ != "unknown" && !removed) {
        if (pose_names_[detection.body_pose] == pose_) {
          RCLCPP_DEBUG(node_->get_logger(), "[IsDetected] Detection %s is %s", detection.unique_id.c_str(), pose_.c_str());
        } else {
          RCLCPP_DEBUG(node_->get_logger(), "[IsDetected] Detection %s is not %s", detection.unique_id.c_str(), pose_.c_str());
          it = detections.erase(it);
          removed = true;
        }
      }

      if (!removed) {
        frames_.push_back(detection.class_name + "_" + std::to_string(entity_counter));

        if (
          pl::getInstance(node_)->publicTF(
            detection, std::to_string(entity_counter)) == -1)
        {
          return BT::NodeStatus::FAILURE;
        }
        entity_counter++;
        ++it;
      }
      
    } else {
      ++it;
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
  // RCLCPP_INFO(node_->get_logger(), "Pointing direction: %d", detections[0].pointing_direction);
  
  RCLCPP_INFO(node_->get_logger(), "[IsDetected] Detections published");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsDetected>("IsDetected");
}