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

#include "motion/navigation/MoveTo.hpp"

#include <fstream>
#include <iostream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace navigation
{

MoveTo::MoveTo(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: motion::BtActionNode<
    nav2_msgs::action::NavigateToPose, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);

  set_truncate_distance_client_ =
    node_->create_client<navigation_system_interfaces::srv::SetTruncateDistance>(
    "navigation_system_node/set_truncate_distance");
}

void MoveTo::on_tick()
{
  RCLCPP_INFO(node_->get_logger(), "MoveTo: on_tick()");
  geometry_msgs::msg::PoseStamped goal;
  geometry_msgs::msg::TransformStamped map_to_goal;

  getInput("tf_frame", tf_frame_);
  getInput("distance_tolerance", distance_tolerance_);
  getInput("will_finish", will_finish_);
  getInput("is_truncated", is_truncated_);

  try {
    map_to_goal = tf_buffer_.lookupTransform("map", tf_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform %s to %s: %s", "map", tf_frame_.c_str(), ex.what());
    setStatus(BT::NodeStatus::RUNNING);
  }

  goal.header.frame_id = "map";
  goal.pose.position.x = map_to_goal.transform.translation.x;
  goal.pose.position.y = map_to_goal.transform.translation.y;
  goal.pose.orientation.x = map_to_goal.transform.rotation.x;
  goal.pose.orientation.y = map_to_goal.transform.rotation.y;
  goal.pose.orientation.z = map_to_goal.transform.rotation.z;
  goal.pose.orientation.w = map_to_goal.transform.rotation.w;

  if (!set_truncate_distance_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node_->get_logger(), "Waiting for action server to be up...");
    setStatus(BT::NodeStatus::RUNNING);
  }

  RCLCPP_INFO(
    //print also the quaternion : 
    node_->get_logger(), "Sending goal: x: %f, y: %f, qx: %f, qy: %f, qz: %f qw: %f  in frame: %s", goal.pose.position.x,
    goal.pose.position.y,
    goal.pose.orientation.x,
    goal.pose.orientation.y,
    goal.pose.orientation.z,
    goal.pose.orientation.w,
     goal.header.frame_id.c_str());

  if (is_truncated_) {
    xml_path_ = generate_xml_file(nav_to_pose_truncated_xml, distance_tolerance_);

  // auto request = std::make_shared<navigation_system_interfaces::srv::SetTruncateDistance::Request>();

  // request->distance = distance_tolerance_;
  // request->xml_content = nav_to_pose_truncated_xml;
  // auto future_request = set_truncate_distance_client_->async_send_request(request).share();
  // if (rclcpp::spin_until_future_complete(node_, future_request) ==
  // rclcpp::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_INFO(node_->get_logger(), "Truncate distance setted");
  //   auto result = *future_request.get();
  //   if (!result.success) {
  //     RCLCPP_ERROR(node_->get_logger(), "Truncate distance FAILED calling service");
  //     setStatus(BT::NodeStatus::FAILURE);
  //   }
  //   xml_path_ = result.xml_path;
  // } else {
  //   RCLCPP_ERROR(node_->get_logger(), "Truncate distance FAILED");
  //   setStatus(BT::NodeStatus::FAILURE);
  // }
  goal_.behavior_tree = xml_path_;
  }
  
  goal_.pose = goal;
}

BT::NodeStatus MoveTo::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "Navigation succeeded");
  if (will_finish_) {
    return BT::NodeStatus::SUCCESS;
  }
  goal_updated_ = true;
  on_tick();
  on_new_goal_received();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveTo::on_aborted()
{
  if (will_finish_) {
    return BT::NodeStatus::FAILURE;
  }
  on_tick();
  on_new_goal_received();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveTo::on_cancelled()
{
  if (will_finish_) {
    return BT::NodeStatus::SUCCESS;
  }
  on_tick();
  on_new_goal_received();
  RCLCPP_INFO(node_->get_logger(), "Navigation cancelled");
  return BT::NodeStatus::RUNNING;
}

}  // namespace navigation

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<navigation::MoveTo>(name, "/navigate_to_pose", config);
    };

  factory.registerBuilder<navigation::MoveTo>("MoveTo", builder);
}
