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

#include "perception/count_people.hpp"

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

CountPeople::CountPeople(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  max_entities_(10),
  colors_({
    {"lower_blue", cv::Scalar(90, 50, 50)}, {"upper_blue", cv::Scalar(125, 255, 255)},
    {"lower_yellow", cv::Scalar(25, 150, 200)}, {"upper_yellow", cv::Scalar(30, 255, 255)},
    {"lower_black", cv::Scalar(0, 0, 0)}, {"upper_black", cv::Scalar(255, 255, 50)},
    {"lower_white", cv::Scalar(0, 0, 200)}, {"upper_white", cv::Scalar(255, 30, 255)},
    {"lower_red", cv::Scalar(170, 150, 180)}, {"upper_red", cv::Scalar(205, 255, 255)},
    {"lower_orange", cv::Scalar(10, 120, 120)}, {"upper_orange", cv::Scalar(20, 255, 255)},
    {"lower_gray", cv::Scalar(0, 0, 100)}, {"upper_gray", cv::Scalar(180, 30, 200)}
  })
{
  config().blackboard->get("node", node_);
  config().blackboard->get("tf_buffer", tf_buffer_);
  config().blackboard->get("tf_listener", tf_listener_);

  gestures_["waving"] = {5, 6, 7};
  gestures_["raising"] = {6};
  gestures_["pointing"] = {0, 1, 3, 4};
}

BT::NodeStatus CountPeople::tick()
{
  rclcpp::spin_some(node_->get_node_base_interface());

  auto num_entities = 0;
  int prev_num_person;

  getInput("color", color_);
  getInput("pose", pose_);
  getInput("max_entities", max_entities_);
  getInput("confidence", threshold_);
  getInput("cam_frame", cam_frame_);
  getInput("input_num_person", prev_num_person);

  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_INFO(node_->get_logger(), "CountPeople idle");
  }

  RCLCPP_INFO(node_->get_logger(), "CountPeople ticked");
  pl::getInstance(node_)->set_interest("person", true);
  pl::getInstance(node_)->update(35);

  auto detections = pl::getInstance(node_)->get_by_type("person");

  if (detections.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[CountPeople] No detections");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(node_->get_logger(), "[CountPeople] Processing %ld detections...", detections.size());
  auto entity_counter = 0;
  for (auto it = detections.begin(); it != detections.end() && entity_counter < max_entities_; ) {
    auto const & detection = *it;
    bool removed = false;

    if (detection.score > threshold_) {
      // Color filtering
      if (color_ != "none") {
        auto const detection_id_colors = perception_system::getHSVFromUniqueID(
          detection.color_person);
        std::string lower_color = "lower_" + color_;
        std::string upper_color = "upper_" + color_;

        cv::Scalar lower_bound = colors_[lower_color];
        cv::Scalar upper_bound = colors_[upper_color];

        cv::Scalar detection_color = detection_id_colors[0];

        double hue = detection_color[0];

        if (color_ == "red") {
          hue += 180;
        }

        RCLCPP_INFO(
          node_->get_logger(), "[CountPeople] Detection %s is %f %f %f",
          detection.unique_id.c_str(), detection_color[0], detection_color[1],
          detection_color[2]);

        if (hue >= lower_bound[0] && hue <= upper_bound[0] &&
          detection_color[1] >= lower_bound[1] && detection_color[1] <= upper_bound[1] &&
          detection_color[2] >= lower_bound[2] && detection_color[2] <= upper_bound[2])
        {
          RCLCPP_INFO(
            node_->get_logger(), "[CountPeople] Detection %s is %s",
            detection.unique_id.c_str(), color_.c_str());
        } else {
          // RCLCPP_INFO(node_->get_logger(), "[CountPeople] Detection %s is not %s", detection.unique_id.c_str(), color_.c_str());
          it = detections.erase(it);
          removed = true;
        }
      }

      // Pose filtering
      if (pose_ != "none" && !removed) {
        if (std::find(
            gestures_[pose_].begin(), gestures_[pose_].end(),
            detection.pointing_direction) != gestures_[pose_].end())
        {
          RCLCPP_INFO(
            node_->get_logger(), "[CountPeople] Detection %s is %s",
            detection.unique_id.c_str(), pose_.c_str());
        } else {
          RCLCPP_INFO(
            node_->get_logger(), "[CountPeople] Detection %s is not %s",
            detection.unique_id.c_str(), pose_.c_str());
          it = detections.erase(it);
          removed = true;
        }
      }

      if (!removed) {
        num_entities++;
        ++it;
      }

    } else {
      ++it;
    }
  }

  RCLCPP_DEBUG(node_->get_logger(), "[CountPeople] Detections sorted and filtered");
  if (frames_.empty() && num_entities == 0) {
    RCLCPP_ERROR(node_->get_logger(), "[CountPeople] No detections after filter");
    // return BT::NodeStatus::SUCCESS;
  }

  setOutput("num_person", prev_num_person + num_entities);

  RCLCPP_INFO(node_->get_logger(), "[CountPeople] %d people detected", num_entities);

  return BT::NodeStatus::SUCCESS;
}

void CountPeople::halt()
{
  RCLCPP_INFO(node_->get_logger(), "CountPeople halted");
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::CountPeople>("CountPeople");
}
