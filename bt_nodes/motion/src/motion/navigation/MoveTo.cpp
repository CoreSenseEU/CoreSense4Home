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

#include <iostream>
#include <fstream>
#include <string>

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
    action_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);

}

void
MoveTo::on_tick()
{
  RCLCPP_INFO(node_->get_logger(), "MoveTo: on_tick()");
  geometry_msgs::msg::PoseStamped goal;
   geometry_msgs::msg::TransformStamped map_to_goal;

  getInput("tf_frame", tf_frame_);
  getInput("distance_tolerance", distance_tolerance_);
  getInput("will_finish", will_finish_);
 
  try {
    map_to_goal = tf_buffer_.lookupTransform(
      "map", tf_frame_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform %s to %s: %s",
      "map", tf_frame_.c_str(), ex.what());
    throw BT::RuntimeError("Could not transform");
  }

  goal.header.frame_id = "map";
  goal.pose.position.x = map_to_goal.transform.translation.x;
  goal.pose.position.y = map_to_goal.transform.translation.y;

  RCLCPP_INFO(
    node_->get_logger(), "Sending goal: x: %f, y: %f, in frame: %s",
    goal.pose.position.x, goal.pose.position.y,
    goal.header.frame_id.c_str());
  

  std::string pkgpath = ament_index_cpp::get_package_share_directory("bt_test");
  std::string xml_file = pkgpath + "/bt_xml/moveto.xml";

  std::ifstream input_file(xml_file);

  if (!input_file.is_open()) {
      std::cerr << "Error opening XML file." << std::endl;
      // setStatus(BT::NodeStatus::FAILURE);
      return;
  }
  std::string xml_content((std::istreambuf_iterator<char>(input_file)), std::istreambuf_iterator<char>());
  input_file.close();

 size_t truncate_pos = xml_content.find("<TruncatePath");
  if (truncate_pos != std::string::npos) {
      size_t distance_pos = xml_content.find("distance=\"", truncate_pos);
      if (distance_pos != std::string::npos) {
          size_t end_qote_pos = xml_content.find("\"", distance_pos + 10);
          if (end_qote_pos != std::string::npos) {
              xml_content.replace(distance_pos + 10, end_qote_pos - distance_pos - 10, std::to_string(distance_tolerance_));
          }
      }
  } else {
      std::cerr << "Element not found in XML." << std::endl;
      // setStatus(BT::NodeStatus::FAILURE);
      return;
  }
  // Save the updated XML back to the same file
  std::ofstream output_file(xml_file);
  if (!output_file.is_open()) {
      std::cerr << "Error opening output file." << std::endl;
      // setStatus(BT::NodeStatus::FAILURE);
      return;
  }

  output_file << xml_content;
  output_file.close();
  
  goal_.behavior_tree = xml_file;
  goal_.pose = goal;
}

BT::NodeStatus
MoveTo::on_success()
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

BT::NodeStatus
MoveTo::on_aborted()
{
  if (will_finish_) {
    return BT::NodeStatus::FAILURE;
  }
  on_tick();
  on_new_goal_received();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus
MoveTo::on_cancelled()
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
  BT::NodeBuilder builder = [](const std::string & name,
      const BT::NodeConfiguration & config) {
      return std::make_unique<navigation::MoveTo>(name, "/navigate_to_pose", config);
    };

  factory.registerBuilder<navigation::MoveTo>("MoveTo", builder);
}
