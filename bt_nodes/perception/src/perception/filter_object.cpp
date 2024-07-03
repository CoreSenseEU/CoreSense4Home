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

#include "perception/filter_object.hpp"

#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace perception {

using namespace std::chrono_literals;
using namespace std::placeholders;

FilterObject::FilterObject(const std::string &xml_tag_name,
                           const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(xml_tag_name, conf),
      objects_({{"7up", {"drink", 10.0f, 0.5f}},
                {"apple", {"fruit", 9.0f, 0.2f}},
                {"bag", {"accessory", 20.0f, 0.3f}},
                {"ball", {"toy", 8.0f, 0.4f}},
                {"banana", {"fruit", 15.0f, 0.25f}},
                {"baseball", {"toy", 7.0f, 0.15f}},
                {"bowl", {"dish", 12.0f, 0.5f}},
                {"cheezit", {"snack", 5.0f, 0.1f}},
                {"chocolate_jello", {"food", 6.0f, 0.2f}},
                {"cleanser", {"cleaning_supply", 15.0f, 0.75f}},
                {"coffe_grounds", {"food", 8.0f, 0.3f}},
                {"cola", {"drink", 10.0f, 0.5f}},
                {"cornflakes", {"snack", 20.0f, 0.4f}},
                {"cup", {"dish", 7.0f, 0.2f}},
                {"dice", {"toy", 2.0f, 0.05f}},
                {"dishwasher_tab", {"cleaning_supply", 1.0f, 0.02f}},
                {"fork", {"dish", 4.0f, 0.1f}},
                {"iced_Tea", {"drink", 10.0f, 0.5f}},
                {"juice_pack", {"drink", 12.0f, 0.4f}},
                {"knife", {"dish", 5.0f, 0.15f}},
                {"lemon", {"fruit", 7.0f, 0.2f}},
                {"milk", {"drink", 10.0f, 1.0f}},
                {"mustard", {"food", 6.0f, 0.3f}},
                {"orange", {"fruit", 9.0f, 0.25f}},
                {"orange_juice", {"drink", 12.0f, 0.5f}},
                {"peach", {"fruit", 8.0f, 0.2f}},
                {"pear", {"fruit", 9.0f, 0.25f}},
                {"person", {"human", 170.0f, 70.0f}},
                {"plate", {"dish", 10.0f, 0.5f}},
                {"plum", {"fruit", 6.0f, 0.15f}},
                {"pringles", {"snack", 20.0f, 0.5f}},
                {"red_wine", {"drink", 10.0f, 1.0f}},
                {"rubiks_cube", {"toy", 5.0f, 0.2f}},
                {"soccer_ball", {"toy", 22.0f, 0.4f}},
                {"spam", {"food", 8.0f, 0.3f}},
                {"sponge", {"cleaning_supply", 10.0f, 0.1f}},
                {"spoon", {"dish", 4.0f, 0.1f}},
                {"strawberry", {"fruit", 3.0f, 0.05f}},
                {"strawberry_jello", {"food", 6.0f, 0.2f}},
                {"sugar", {"food", 8.0f, 0.5f}},
                {"tennis_ball", {"toy", 7.0f, 0.2f}},
                {"tomato_soup", {"food", 10.0f, 0.5f}},
                {"tropical_juice", {"drink", 12.0f, 0.5f}},
                {"tuna", {"food", 7.0f, 0.3f}},
                {"bottle", {"drink", 10.0f, 1.0f}},
                {"water", {"drink", 10.0f, 1.0f}}}) {
  config().blackboard->get("node", node_);

  RCLCPP_INFO(node_->get_logger(), "FilterObject initialized");
}

void FilterObject::halt() {
  RCLCPP_INFO(node_->get_logger(), "FilterObject halted");
}

BT::NodeStatus FilterObject::tick() {

  RCLCPP_INFO(node_->get_logger(), "FilterObject ticked");

  getInput("frames", frames_);

  frames_ = extractClassNames(frames_);

  for (const auto &frame : frames_) {
    RCLCPP_INFO(node_->get_logger(), "[FilterObject] Detected object: %s",
                frame.c_str());
  }

  getInput("size", size_);
  getInput("weight", weight_);
  getInput("class", class_);

  if ((size_ == "unknown" || size_.empty()) && (weight_ == "unknown" || weight_.empty()) && (class_ == "unknown" || class_.empty()) ) {
    RCLCPP_INFO(node_->get_logger(), "[FilterObject] No filter specified");
    return BT::NodeStatus::SUCCESS;
  }

  if (class_ != "unknown" && !class_.empty()) {
    RCLCPP_INFO(node_->get_logger(),
                "[FilterObject] Filtering object by class: %s", class_.c_str());

    config().blackboard->set("out_msg", class_);

    for (auto &object : frames_) {
      RCLCPP_INFO(node_->get_logger(), "[FilterObject] %s",
                  objects_[object].class_type.c_str());
      if (objects_[object].class_type == class_) {
        filtered_objects_.push_back(object);
      }
    }
  }

  if (!filtered_objects_.empty()) {
    frames_ = filtered_objects_;
    objects_count_ = std::to_string(filtered_objects_.size());
    setOutput("objects_count", objects_count_);
    RCLCPP_INFO(node_->get_logger(), "[FilterObject] %s detections of %s",
                objects_count_.c_str(), class_.c_str());
    filtered_object_ = filtered_objects_[0];
  }

  filtered_objects_.clear();

  if (size_ != "unknown" && !size_.empty()) {
    RCLCPP_INFO(node_->get_logger(),
                "[FilterObject] Filtering object by size: %s", size_.c_str());
    if (size_ == "big" || size_ == "large") {
      filtered_object_ =
          getObject(frames_, std::greater<float>(), &ObjectInfo::size);
    } else {
      filtered_object_ =
          getObject(frames_, std::less<float>(), &ObjectInfo::size);
    }
  }

  if (weight_ != "unknown" && !weight_.empty()) {
    RCLCPP_INFO(node_->get_logger(),
                "[FilterObject] Filtering object by weight: %s",
                weight_.c_str());
    if (weight_ == "heavy") {
      filtered_object_ =
          getObject(frames_, std::greater<float>(), &ObjectInfo::weight);
    } else {
      filtered_object_ =
          getObject(frames_, std::less<float>(), &ObjectInfo::weight);
    }
  }

  if (!filtered_object_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[FilterObject] The object filtered is %s",
                filtered_object_.c_str());

    setOutput("filtered_object", filtered_object_);
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

std::string FilterObject::getObject(const std::vector<std::string> &frames,
                                    std::function<bool(float, float)> compare,
                                    float ObjectInfo::*info) {
  float limit = compare(0.0f, std::numeric_limits<float>::max())
                    ? std::numeric_limits<float>::max()
                    : 0.0f;
  std::string filtered_object;

  for (auto &object : frames) {
    if (compare(objects_[object].*info, limit)) {
      limit = objects_[object].*info;
      filtered_object = object;
    }
  }

  return filtered_object;
}

std::vector<std::string>
FilterObject::extractClassNames(const std::vector<std::string> &frames) {
  std::vector<std::string> classNames;
  for (const auto &frame : frames) {
    size_t pos = frame.find('_');
    if (pos != std::string::npos) {
      std::string className = frame.substr(0, pos);
      std::transform(className.begin(), className.end(), className.begin(),
                     ::tolower);
      classNames.push_back(className);
    }
  }
  return classNames;
}

} // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::FilterObject>("FilterObject");
}
