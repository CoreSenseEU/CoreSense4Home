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

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

FilterObject::FilterObject(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  objects_({{"person", {"human", 170.0f, 70.0f}},
    {"7up", {"drink", 12.0f, 0.36f}},
    {"cola", {"drink", 12.0f, 0.36f}},
    {"water", {"drink", 20.0f, 0.50f}},
    {"milk", {"drink", 20.0f, 1.00f}},
    {"orange juice", {"drink", 20.0f, 1.00f}},
    {"tropical juice", {"drink", 20.0f, 1.00f}},
    {"red wine", {"drink", 30.0f, 1.50f}},
    {"redbull", {"drink", 15.0f, 0.25f}},
    {"iced tea", {"drink", 20.0f, 0.50f}},
    {"juice pack", {"drink", 10.0f, 0.25f}},
    {"pringles", {"snack", 25.0f, 0.15f}},
    {"cheezit", {"snack", 15.0f, 0.35f}},
    {"cornflakes", {"snack", 30.0f, 0.50f}},
    {"tennis ball", {"toy", 6.5f, 0.06f}},
    {"dice", {"toy", 2.0f, 0.01f}},
    {"soccer ball", {"toy", 22.0f, 0.43f}},
    {"baseball", {"toy", 7.0f, 0.15f}},
    {"rubiks cube", {"toy", 5.7f, 0.10f}},
    {"knife", {"dish", 20.0f, 0.10f}},
    {"fork", {"dish", 20.0f, 0.09f}},
    {"bowl", {"dish", 15.0f, 0.35f}},
    {"plate", {"dish", 25.0f, 0.50f}},
    {"cup", {"dish", 10.0f, 0.20f}},
    {"spoon", {"dish", 15.0f, 0.08f}},
    {"sugar", {"food", 15.0f, 1.00f}},
    {"coffee grounds", {"food", 15.0f, 0.50f}},
    {"strawberry jello", {"food", 10.0f, 0.17f}},
    {"chocolate jello", {"food", 10.0f, 0.17f}},
    {"spam", {"food", 10.0f, 0.34f}},
    {"tomato soup", {"food", 10.0f, 0.30f}},
    {"mustard", {"food", 15.0f, 0.22f}},
    {"tuna", {"food", 5.0f, 0.17f}},
    {"plum", {"fruit", 5.0f, 0.08f}},
    {"pear", {"fruit", 8.0f, 0.18f}},
    {"apple", {"fruit", 8.0f, 0.20f}},
    {"lemon", {"fruit", 6.0f, 0.10f}},
    {"peach", {"fruit", 7.0f, 0.15f}},
    {"strawberry", {"fruit", 3.0f, 0.02f}},
    {"banana", {"fruit", 18.0f, 0.15f}},
    {"orange", {"fruit", 8.0f, 0.20f}},
    {"toothpaste", {"cleaning supply", 15.0f, 0.10f}},
    {"cleanser", {"cleaning supply", 20.0f, 0.50f}},
    {"sponge", {"cleaning supply", 10.0f, 0.02f}},
  })
{
  config().blackboard->get("node", node_);

  RCLCPP_INFO(node_->get_logger(), "FilterObject initialized");
}

void FilterObject::halt()
{
  RCLCPP_INFO(node_->get_logger(), "FilterObject halted");
}

BT::NodeStatus FilterObject::tick()
{

  RCLCPP_INFO(node_->get_logger(), "FilterObject ticked");

  getInput("frames", frames_);

  frames_ = extractClassNames(frames_);

  for (const auto & frame : frames_) {
    RCLCPP_INFO(
      node_->get_logger(), "[FilterObject] Detected object: %s",
      frame.c_str());
  }

  getInput("size", size_);
  getInput("weight", weight_);
  getInput("class", class_);

  if ((size_ == "unknown" || size_.empty()) &&
    (weight_ == "unknown" || weight_.empty()) &&
    (class_ == "unknown" || class_.empty()))
  {
    RCLCPP_INFO(node_->get_logger(), "[FilterObject] No filter specified");
    return BT::NodeStatus::SUCCESS;
  }

  if (class_ != "unknown" && !class_.empty()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[FilterObject] Filtering object by class: %s", class_.c_str());

    config().blackboard->set("out_msg", class_);

    for (auto & object : frames_) {
      RCLCPP_INFO(
        node_->get_logger(), "[FilterObject] %s",
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
    RCLCPP_INFO(
      node_->get_logger(), "[FilterObject] %s detections of %s",
      objects_count_.c_str(), class_.c_str());
    filtered_object_ = filtered_objects_[0];
  }

  filtered_objects_.clear();

  if (size_ != "unknown" && !size_.empty()) {
    RCLCPP_INFO(
      node_->get_logger(),
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
    RCLCPP_INFO(
      node_->get_logger(),
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
    RCLCPP_INFO(
      node_->get_logger(), "[FilterObject] The object filtered is %s",
      filtered_object_.c_str());

    setOutput("filtered_object", filtered_object_);
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

std::string FilterObject::getObject(
  const std::vector<std::string> & frames,
  std::function<bool(float, float)> compare,
  float ObjectInfo::* info)
{
  float limit = compare(0.0f, std::numeric_limits<float>::max()) ?
    std::numeric_limits<float>::max() :
    0.0f;
  std::string filtered_object;

  for (auto & object : frames) {
    if (compare(objects_[object].*info, limit)) {
      limit = objects_[object].*info;
      filtered_object = object;
    }
  }

  return filtered_object;
}

std::vector<std::string>
FilterObject::extractClassNames(const std::vector<std::string> & frames)
{
  std::vector<std::string> classNames;
  for (const auto & frame : frames) {
    size_t pos = frame.find('_');
    if (pos != std::string::npos) {
      std::string className = frame.substr(0, pos);
      std::transform(
        className.begin(), className.end(), className.begin(),
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
