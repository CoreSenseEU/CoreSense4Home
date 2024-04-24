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

#ifndef PERCEPTION__ISDETECTED_HPP_
#define PERCEPTION__ISDETECTED_HPP_

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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace perception
{

using pl = perception_system::PerceptionListener;

class IsDetected : public BT::ConditionNode
{
public:
  explicit IsDetected(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<int>("max_entities"),
        BT::InputPort<int>("person_id"),
        BT::InputPort<std::string>("cam_frame"),
        BT::InputPort<std::string>("interest"),
        BT::InputPort<float>("confidence"),
        BT::InputPort<std::string>("order"), // todo: enum map or string?
        BT::InputPort<double>("max_depth"),
        BT::OutputPort<std::vector<std::string>>("frames")
      });
  }

private:
  int publicTF_map2object(
    const perception_system_interfaces::msg::Detection & detected_object,
    const std::string & frame_name);

  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string interest_, order_, cam_frame_;
  double threshold_, max_depth_;
  int max_entities_, person_id_;
  std::vector<std::string> frames_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace perception

#endif  // PERCEPTION__ISDETECTED_HPP_
