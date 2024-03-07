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

#ifndef PERCEPTION__FOLLOW_PERSON_HPP_
#define PERCEPTION__FOLLOW_PERSON_HPP_

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

#include "rclcpp/rclcpp.hpp"

#include "perception_system/PerceptionListener.hpp"

namespace perception
{

class FollowPerson : public BT::ActionNodeBase
{
public:
  explicit FollowPerson(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::int64_t>("person_id"),
        BT::InputPort<std::string>("camera_link"),
      });
  }

private:
  int publicTF_map2object(const perception_system_interfaces::msg::Detection & detected_object);

  rclcpp::Node::SharedPtr node_;

  std::string camera_link_;
  int64_t person_id_;
  geometry_msgs::msg::TransformStamped person_pose_;
  rclcpp::Time last_pose_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

}  // namespace perception

#endif  // PERCEPTION__FOLLOW_PERSON_HPP_
