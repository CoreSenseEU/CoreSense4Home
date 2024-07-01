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
#include "perception_system/PerceptionUtils.hpp"

namespace perception {

using namespace std::chrono_literals;
using namespace std::placeholders;

using pl = perception_system::PerceptionListener;

FilterObject::FilterObject(const std::string &xml_tag_name,
                           const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(xml_tag_name, conf),
      objects_({{"7up", {"drink", 10.0f, 0.5f}},
                {"apple", {"fruit", 9.0f, 0.2f}},
                {"Bag", {"accessory", 20.0f, 0.3f}},
                {"Ball", {"toy", 8.0f, 0.4f}},
                {"Banana", {"fruit", 15.0f, 0.25f}},
                {"Baseball", {"toy", 7.0f, 0.15f}},
                {"Bowl", {"dish", 12.0f, 0.5f}},
                {"Cheezit", {"snack", 5.0f, 0.1f}},
                {"Chocolate_jello", {"food", 6.0f, 0.2f}},
                {"Cleanser", {"cleaning_supply", 15.0f, 0.75f}},
                {"Coffe_grounds", {"food", 8.0f, 0.3f}},
                {"Cola", {"drink", 10.0f, 0.5f}},
                {"Cornflakes", {"snack", 20.0f, 0.4f}},
                {"Cup", {"dish", 7.0f, 0.2f}},
                {"Dice", {"toy", 2.0f, 0.05f}},
                {"Dishwasher_tab", {"cleaning_supply", 1.0f, 0.02f}},
                {"Fork", {"dish", 4.0f, 0.1f}},
                {"Iced_Tea", {"drink", 10.0f, 0.5f}},
                {"Juice_pack", {"drink", 12.0f, 0.4f}},
                {"Knife", {"dish", 5.0f, 0.15f}},
                {"Lemon", {"fruit", 7.0f, 0.2f}},
                {"Milk", {"drink", 10.0f, 1.0f}},
                {"Mustard", {"food", 6.0f, 0.3f}},
                {"Orange", {"fruit", 9.0f, 0.25f}},
                {"Orange_juice", {"drink", 12.0f, 0.5f}},
                {"Peach", {"fruit", 8.0f, 0.2f}},
                {"Pear", {"fruit", 9.0f, 0.25f}},
                {"Person", {"human", 170.0f, 70.0f}},
                {"Plate", {"dish", 10.0f, 0.5f}},
                {"Plum", {"fruit", 6.0f, 0.15f}},
                {"Pringles", {"snack", 20.0f, 0.5f}},
                {"Red_wine", {"drink", 10.0f, 1.0f}},
                {"Rubiks_cube", {"toy", 5.0f, 0.2f}},
                {"Soccer_ball", {"toy", 22.0f, 0.4f}},
                {"Spam", {"food", 8.0f, 0.3f}},
                {"Sponge", {"cleaning_supply", 10.0f, 0.1f}},
                {"Spoon", {"dish", 4.0f, 0.1f}},
                {"Strawberry", {"fruit", 3.0f, 0.05f}},
                {"Strawberry_jello", {"food", 6.0f, 0.2f}},
                {"Sugar", {"food", 8.0f, 0.5f}},
                {"Tennis_ball", {"toy", 7.0f, 0.2f}},
                {"Tomato_soup", {"food", 10.0f, 0.5f}},
                {"Tropical_juice", {"drink", 12.0f, 0.5f}},
                {"Tuna", {"food", 7.0f, 0.3f}},
                {"Water", {"drink", 10.0f, 1.0f}}}) {
  config().blackboard->get("node", node_);

  getInput("cam_frame", camera_frame_);
  config().blackboard->get("tf_buffer", tf_buffer_);
  config().blackboard->get("tf_static_broadcaster", tf_static_broadcaster_);

  RCLCPP_INFO(node_->get_logger(), "FilterObject initialized");
}

void FilterObject::halt() {
  RCLCPP_INFO(node_->get_logger(), "FilterObject halted");
}

BT::NodeStatus FilterObject::tick() {
  getInput("frames", frames_);

  RCLCPP_INFO(node_->get_logger(), "FilterObject ticked");

  frames_ = extractClassNames(frames_);

  for (const auto &frame : frames_) {
    RCLCPP_INFO(node_->get_logger(), "[FilterObject] Detected object: %s",
                frame.c_str());
  }

  getInput("size", size_);
  getInput("weight", weight_);
  getInput("class", class_);

  if (size_ != "unknown") {
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

  if (weight_ != "unknown") {
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

  if (class_ != "unknown") {
    RCLCPP_INFO(node_->get_logger(),
                "[FilterObject] Filtering object by class: %s", class_.c_str());

    for (const auto &object : frames_) {
      if (objects_[object].class_type == class_) {
        filtered_object_ = object;
      }
    }
  }

  if (!filtered_object_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "[FilterObject] The object filtered is %s",
                filtered_object_.c_str());

    // pl::getInstance(node_)->set_interest(filtered_object_.c_str(), true);
    // pl::getInstance(node_)->update(true);

    // std::vector<perception_system_interfaces::msg::Detection> detections;
    // detections = pl::getInstance(node_)->get_by_type(filtered_object_.c_str());

    // if (detections.empty()) {
    //   RCLCPP_INFO(node_->get_logger(), "[FilterObject] No detect %s",
    //               filtered_object_.c_str());
    //   return BT::NodeStatus::FAILURE;
    // }

    // int dev = publicTF_map2object(detections[0]);

    setOutput("filtered_object", filtered_object_);
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

int FilterObject::publicTF_map2object(
    const perception_system_interfaces::msg::Detection &detected_object) {
  geometry_msgs::msg::TransformStamped map2camera_msg;
  perception_system_interfaces::msg::Detection modified_detection;
  modified_detection = detected_object;

  RCLCPP_INFO(node_->get_logger(), "[FilterObject] Detected object frame_id %s",
              detected_object.header.frame_id.c_str());

  try {
    map2camera_msg = tf_buffer_->lookupTransform("map", "base_footprint",
                                                 tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(node_->get_logger(),
                "[FilterObject] Could not transform %s to %s: %s", "map",
                camera_frame_.c_str(), ex.what());
    return -1;
  }

  tf2::Transform camera2object;
  camera2object.setOrigin(tf2::Vector3(modified_detection.center3d.position.x,
                                       modified_detection.center3d.position.y,
                                       modified_detection.center3d.position.z));
  camera2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  tf2::Transform map2camera;
  tf2::fromMsg(map2camera_msg.transform, map2camera);

  tf2::Transform map2object = map2camera * camera2object;
  // create a transform message from tf2::Transform
  geometry_msgs::msg::TransformStamped map2object_msg;
  map2object_msg.header.frame_id = "map";
  map2object_msg.child_frame_id = detected_object.class_id;

  map2object_msg.transform = tf2::toMsg(map2object);
  map2object_msg.transform.translation.z = 0.0;

  tf_static_broadcaster_->sendTransform(map2object_msg);
  return 0;
}

std::string FilterObject::getObject(const std::vector<std::string> &frames,
                                    std::function<bool(float, float)> compare,
                                    float ObjectInfo::*info) {
  float limit = compare(0.0f, std::numeric_limits<float>::max())
                    ? std::numeric_limits<float>::max()
                    : 0.0f;
  std::string filtered_object;

  for (const auto &object : frames) {
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
      classNames.push_back(frame.substr(0, pos));
    }
  }
  return classNames;
}

} // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::FilterObject>("FilterObject");
}
