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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace navigation
{

MoveTo::MoveTo(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: navigation::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name,
 action_name, conf)
 {
  config().blackboard->get("node", node_);
 }

void
MoveTo::on_tick()
{
  RCLCPP_INFO(node_->get_logger(), "MoveTo: on_tick()");
  geometry_msgs::msg::PoseStamped goal;
  getInput("tf_frame",tf_frame_);
  config().blackboard->get(tf_frame_, goal);

  RCLCPP_INFO(node_->get_logger(), "Sending goal: x: %f, y: %f, in frame: %s",
  goal.pose.position.x, goal.pose.position.y,
      goal.header.frame_id.c_str());
  goal_.pose = goal;

  std::string pkgpath = ament_index_cpp::get_package_share_directory("bt_test");
  std::string xml_file = pkgpath + "/bt_xml/moveto.xml";
  goal_.behavior_tree = xml_file;
  on_new_goal_received();
}

BT::NodeStatus
MoveTo::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "Navigation succeeded");

  return BT::NodeStatus::SUCCESS;
}


}  // namespace navigation

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
BT::NodeBuilder builder = [](const std::string &name,
                               const BT::NodeConfiguration & config) {
    return std::make_unique<navigation::MoveTo>(name, "navigate_to_pose", config);
  };

  factory.registerBuilder<navigation::MoveTo>("MoveTo",builder);
}