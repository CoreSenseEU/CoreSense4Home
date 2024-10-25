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

#ifndef ARM_MANIPULATION__GENERATE_GRASP_POSES_HPP_
#define ARM_MANIPULATION__GENERATE_GRASP_POSES_HPP_

#include <algorithm>
#include <string>

#include "arm/manipulation/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "manipulation_interfaces/action/generate_grasp_poses.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace manipulation
{

class GenerateGraspPoses
  : public manipulation::BtActionNode<
    manipulation_interfaces::action::GenerateGraspPoses,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit GenerateGraspPoses(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<sensor_msgs::msg::PointCloud2>("object_to_pick"),
        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("grasp_poses"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("best_pose")});
  }
};

}  // namespace manipulation

#endif  // arm_MANIPULATION__GENERATE_GRASP_POSES_HPP_
