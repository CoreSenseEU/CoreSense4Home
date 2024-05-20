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

#ifndef PERCEPTION__EXTRACT_CABINET_PLACE_POSE_HPP_
#define PERCEPTION__EXTRACT_CABINET_PLACE_POSE_HPP_

#include <algorithm>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "perception/bt_service_node.hpp"
#include "perception_system_interfaces/srv/isolate_pc_background.hpp"
#include "perception_system/PerceptionUtils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <pcl/filters/crop_box.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/buffer.h>
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"


namespace perception
{

class ExtractCabinetPlacePose
  : public perception::BtServiceNode<perception_system_interfaces::srv::IsolatePCBackground,
  rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit ExtractCabinetPlacePose(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  void on_result() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<moveit_msgs::msg::CollisionObject::SharedPtr>("selected_object"),
       BT::OutputPort<geometry_msgs::msg::PoseStamped>("place_pose")});
  }

private:
  moveit_msgs::msg::CollisionObject::SharedPtr selected_object_;
  geometry_msgs::msg::PoseStamped place_pose_;
  pcl::CropBox<pcl::PointXYZRGB> crop_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace perception

#endif  // PERCEPTION__EXTRACT_CABINET_PLACE_POSE_HPP_
