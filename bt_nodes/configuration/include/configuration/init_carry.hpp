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

#ifndef CONFIGURATION__INIT_CARRY_HPP_
#define CONFIGURATION__INIT_CARRY_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace configuration
{

class InitCarry : public BT::ActionNodeBase
{
public:
  explicit InitCarry(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<std::string>("cam_frame", "frame to transform to all detections"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("home_position", "position to  return"),
        BT::OutputPort<std::string>("home_pose", "arm default pose"),
        BT::OutputPort<std::string>("offer_pose", "arm offering pose"),
        BT::OutputPort<int>("person_id", "person id by color detection in HSV"),
        BT::OutputPort<double>("x_axis_max", "max x axis"),
        BT::OutputPort<double>("x_axis_min", "min x axis"),
        BT::OutputPort<double>("y_axis_max", "max y axis"),
        BT::OutputPort<double>("y_axis_min", "min y axis"),
      });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  std::string cam_frame_, home_pose_, offer_pose_;
  int person_id;
  double x_axis_max_, x_axis_min_, y_axis_max_, y_axis_min_;
  geometry_msgs::msg::PoseStamped home_position_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace configuration

#endif  // CONFIGURATION__INIT_CARRY_HPP_
