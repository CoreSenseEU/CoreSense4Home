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

#include <string>
#include <iostream>

#include "motion/head/Pan.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace head
{

Pan::Pan(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: motion::BtActionNode<
  control_msgs::action::FollowJointTrajectory, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
  config().blackboard->get("node", node_);
}


void
Pan::on_tick()
{
  rclcpp::spin_some(node_);
  BT::Optional<std::string> frame_to_pan = getInput<std::string>("tf_frame");

  if (!frame_to_pan) {
    RCLCPP_ERROR(node_->get_logger(), "Pan: tf_frame is missing");
    return;
  }

  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_INFO(node_->get_logger(), "Pan: tf_frame %s", frame_to_pan.value().c_str());
    goal_.trajectory.joint_names = std::vector<std::string>{"head_1_joint", "head_2_joint"};
    // trajectory_msgs::msg::JointTrajectoryPoint point;
    // point.positions = std::vector<double>{point_to_pan_.value(), 0.0};
    // point.time_from_start = rclcpp::Duration::from_seconds(5.0);

    // goal_.trajectory.points.push_back(point);
     
  }
}

void
Pan::on_feedback()
{
  RCLCPP_INFO(node_->get_logger(), "Pan: Feedback");
  control_msgs::action::FollowJointTrajectory::Feedback::SharedPtr feedback;
  config().blackboard->get("feedback", feedback);
  trajectory_msgs::msg::JointTrajectoryPoint actual = feedback->actual;

}

BT::NodeStatus
Pan::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "Pan: Succeeded");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace head

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name,
    const BT::NodeConfiguration & config) {
      return std::make_unique<head::Pan>(
        name, "/head_controller/follow_joint_trajectory", config);
    };

  factory.registerBuilder<head::Pan>(
    "Pan", builder);

}
