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
#include <vector>

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

namespace configuration
{

class InitReceptionist : public BT::ActionNodeBase
{
public:
  explicit InitReceptionist(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::OutputPort<std::string>("cam_frame", "frame to transform to all detections"),
        BT::OutputPort<std::string>("manipulation_frame", "base_frame for manipulation"),
        BT::OutputPort<std::string>("party_wp", "frame to the party wp"),
        BT::OutputPort<std::string>("entrance_wp", "frame to the the entrance wp"),
        BT::OutputPort<std::string>("host_name", "name of the host"),
        BT::OutputPort<std::string>("host_drink", "host's favorite drink"),
        BT::OutputPort<std::string>("current_guest", "current guest_number STARTING FROM 1"),
        BT::OutputPort<std::string>("attention_home", "fixed frame to look at"),
        BT::OutputPort<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"),
        BT::OutputPort<std::shared_ptr<tf2_ros::StaticTransformBroadcaster>>(
          "tf_static_broadcaster"),
        BT::OutputPort<std::shared_ptr<tf2_ros::TransformBroadcaster>>("tf_broadcaster")});
  }

private:
  std::string cam_frame_, manipulation_frame_, host_name_, host_drink_;
  std::vector<double> party_wp_, entrance_wp_;
  std::vector<std::string> wp_names_;

  geometry_msgs::msg::PoseStamped home_position_;

  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace configuration

#endif  // CONFIGURATION__INIT_CARRY_HPP_
