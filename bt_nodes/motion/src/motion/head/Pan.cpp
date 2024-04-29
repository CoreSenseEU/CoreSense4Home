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

#include "motion/head/Pan.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"


namespace head
{

using namespace std::chrono_literals;

Pan::Pan(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: motion::BtActionNode<
    control_msgs::action::FollowJointTrajectory, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
  config().blackboard->get("node", node_);
  joint_range_ = 20.0 * M_PI / 180.0;
  joint_range_ = getInput<double>("range").value() * M_PI / 180.0;
  period_ = getInput<double>("period").value();


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
Pan::halt()
{
  auto client = node_->create_client<lifecycle_msgs::srv::ChangeState>("/attention_server/change_state");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(node_->get_logger(), "[Pan] waiting for service '/attention_server/change_state' to appear...");
  }
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
  auto result = client->async_send_request(request);
  rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result);
}

double
Pan::get_joint_yaw(double period, double range, double time)
{
  return range * sin((2 * M_PI / period) * time);
}

BT::NodeStatus
Pan::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[Pan] ticked");
  if (status() == BT::NodeStatus::IDLE) {
    start_time_ = node_->now();
    auto client = node_->create_client<lifecycle_msgs::srv::ChangeState>("/attention_server/change_state");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(node_->get_logger(), "[Pan] waiting for service '/attention_server/change_state' to appear...");
    }
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    auto result = client->async_send_request(request);
    rclcpp::spin_until_future_complete(node_->get_node_base_interface(), result);
    return BT::NodeStatus::RUNNING;
  }

  trajectory_msgs::msg::JointTrajectory command_msg;
  auto elapsed = node_->now() - start_time_;

  double yaw = get_joint_yaw(period_, joint_range_, elapsed.seconds());

  command_msg.joint_names = std::vector<std::string>{"head_1_joint", "head_2_joint"};
  command_msg.points.resize(1);
  command_msg.points[0].positions.resize(2);
  command_msg.points[0].velocities.resize(2);
  command_msg.points[0].accelerations.resize(2);
  command_msg.points[0].positions[0] = yaw;
  command_msg.points[0].positions[1] = 0.0;
  command_msg.points[0].time_from_start = rclcpp::Duration::from_seconds(1.5);
  joint_cmd_pub_->publish(command_msg);

  return BT::NodeStatus::RUNNING;
}

}  // namespace motion

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
