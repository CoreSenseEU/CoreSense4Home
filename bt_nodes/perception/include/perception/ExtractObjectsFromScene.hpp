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
// See the License for the specific language governing permissions andGO2OBJECT
// limitations under the License.

#ifndef PERCEPTION__EXTRACT_OBJECTS_FROM_SCENE_HPP_
#define PERCEPTION__EXTRACT_OBJECTS_FROM_SCENE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <functional>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"

namespace perception
{

class ExtractObjectsFromScene : public BT::ActionNodeBase
{
public:
  explicit ExtractObjectsFromScene(
    const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::OutputPort<std::vector<moveit_msgs::msg::CollisionObject::SharedPtr>>(
          "detected_objects"),
        BT::OutputPort<size_t>("objects_count")});
  }

  void detection_callback_(yolov8_msgs::msg::DetectionArray::UniquePtr msg);

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr detected_objs_sub_;
  yolov8_msgs::msg::DetectionArray::UniquePtr last_detected_objs_ = {nullptr};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace perception

#endif  // PERCEPTION__EXTRACT_OBJECTS_FROM_SCENE_HPP_
