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

#ifndef PERCEPTION__CHECK_POLICY_HPP_
#define PERCEPTION__CHECK_POLICY_HPP_

#include <pcl/filters/crop_box.h>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "perception/bt_service_node.hpp"
#include "perception_system/PerceptionUtils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vqa_interfaces/srv/answer_boolean_question.hpp"

namespace perception
{

class CheckPolicy : public perception::BtServiceNode<
    vqa_interfaces::srv::AnswerBooleanQuestion, rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit CheckPolicy(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  void on_result() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<std::string>("question"),
        BT::OutputPort<bool>("output")});
  }

private:
  std::string question_;
  bool output_;
};

}  // namespace perception

#endif  // PERCEPTION__CHECK_POLICY_HPP_
