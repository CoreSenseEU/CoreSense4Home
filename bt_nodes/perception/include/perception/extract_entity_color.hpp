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

#ifndef PERCEPTION__EXTRACT_COLOR_HPP_
#define PERCEPTION__EXTRACT_COLOR_HPP_

#include <string>
#include <algorithm>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2_ros/transform_broadcaster.h"

#include "perception_system/PerceptionListener.hpp"
#include "perception_system_interfaces/msg/detection_array.hpp"

#include "rclcpp/rclcpp.hpp"

namespace perception
{

using pl = perception_system::PerceptionListener;

class ExtractEntityColor : public BT::ActionNodeBase
{
public:
  explicit ExtractEntityColor(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("interest"),
        BT::InputPort<float>("confidence"),
        BT::OutputPort<int>("person_id")
      });
  }

private:

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<perception_system_interfaces::msg::DetectionArray>::SharedPtr detected_objs_sub_;
  perception_system_interfaces::msg::DetectionArray::SharedPtr last_detected_objs_ = {nullptr};

  std::string interest_;
  float threshold_;
  int person_id_;
};

}  // namespace perception

#endif  // PERCEPTION__EXTRACT_COLOR_HPP_
