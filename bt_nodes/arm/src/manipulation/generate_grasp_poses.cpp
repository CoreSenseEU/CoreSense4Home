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

#include "arm/manipulation/generate_grasp_poses.hpp"

#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "manipulation_interfaces/action/pick.hpp"

namespace manipulation
{

using namespace std::chrono_literals;
using namespace std::placeholders;

GenerateGraspPoses::GenerateGraspPoses(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: manipulation::BtActionNode<
    manipulation_interfaces::action::GenerateGraspPoses,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
}

void GenerateGraspPoses::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "GenerateGraspPoses ticked");
  sensor_msgs::msg::PointCloud2 pc;
  getInput("object_to_pick", pc);
  goal_.object_goal = pc;
}

BT::NodeStatus GenerateGraspPoses::on_success()
{
  if (result_.result->success) {
    setOutput("grasp_poses", result_.result->poses);
    RCLCPP_INFO(node_->get_logger(), "GenerateGraspPoses succeeded");
    // print the best pose header and position
    RCLCPP_INFO(
      node_->get_logger(), "Best pose header: %s",
      result_.result->poses[0].header.frame_id.c_str());
    RCLCPP_INFO(
      node_->get_logger(), "Best pose position: %f %f %f", result_.result->poses[0].pose.position.x,
      result_.result->poses[0].pose.position.y, result_.result->poses[0].pose.position.z);
    setOutput("best_pose", result_.result->poses[0]);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "GenerateGraspPoses failed");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace manipulation
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<manipulation::GenerateGraspPoses>(
        name, "/generate_grasp_poses",
        config);
    };

  factory.registerBuilder<manipulation::GenerateGraspPoses>("GenerateGraspPoses", builder);
}
