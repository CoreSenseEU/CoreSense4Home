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
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_behavior_tree/plugins/action/follow_path_action.hpp"
#include "nav2_behavior_tree/plugins/action/truncate_path_action.hpp"
#include "nav2_behavior_tree/plugins/action/compute_path_to_pose_action.hpp"
#include "nav_msgs/msg/path.hpp"

#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("moveto_test");
  RCLCPP_INFO(node->get_logger(), "Node created");
  BT::BehaviorTreeFactory factory;
  // static std::shared_ptr<BT::BehaviorTreeFactory> factory = std::make_shared<BT::BehaviorTreeFactory>();  
  RCLCPP_INFO(node->get_logger(), "factory created");
  BT::NodeBuilder cp_builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::ComputePathToPoseAction>(
          name, "compute_path_to_pose", config);
      };

  RCLCPP_INFO(node->get_logger(), "cp builder created");
  factory.registerBuilder<nav2_behavior_tree::ComputePathToPoseAction>(
    "ComputePathToPose", cp_builder);
  
  RCLCPP_INFO(node->get_logger(), "cp builder registered");

  BT::NodeBuilder tp_builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::TruncatePath>(
          name, config);
      };

  factory.registerBuilder<nav2_behavior_tree::TruncatePath>(
    "TruncatePath", tp_builder);
    

  BT::NodeBuilder fp_builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<nav2_behavior_tree::FollowPathAction>(
          name, "follow_path", config);
      };

  factory.registerBuilder<nav2_behavior_tree::FollowPathAction>(
    "FollowPath", fp_builder);
  
  RCLCPP_INFO(node->get_logger(), "Bt plugins nodes loaded");

  std::string pkgpath = ament_index_cpp::get_package_share_directory("motion");
  std::string xml_file = pkgpath + "/bt_xml/moveto2.xml";

  auto blackboard = BT::Blackboard::create();
  RCLCPP_INFO(node->get_logger(), "Blackboard created");
  blackboard->set("node", node);
  RCLCPP_INFO(node->get_logger(), "Node set in blackboard");

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 1.0;
  pose.pose.position.z = 0.0;
  blackboard->set("goal", pose);
  RCLCPP_INFO(node->get_logger(), "Goal set in blackboard");
/*

  int timeout;
  node->get_parameter("bt_loop_duration", timeout);
  auto bt_loop_duration = std::chrono::milliseconds(timeout);
  node->get_parameter("default_server_timeout", timeout);
  auto default_server_timeout = std::chrono::milliseconds(timeout);

  blackboard->set<std::chrono::milliseconds>("server_timeout", default_server_timeout); 
  blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration); 
*/
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);
  RCLCPP_INFO(node->get_logger(), "Tree created");

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;

    rclcpp::spin_some(node);
    RCLCPP_INFO(node->get_logger(), "Tree Ticked and node spinned");
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
