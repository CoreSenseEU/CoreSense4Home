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

#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    "isdetected_test");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("is_detected_bt_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("bt_test");
  std::string xml_file = pkgpath + "/bt_xml/isdetected_test.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped static_transform_stamped;
  static_transform_stamped.header.stamp = node->now();
  static_transform_stamped.header.frame_id = "head_front_camera_link";
  static_transform_stamped.child_frame_id = "person";
  static_transform_stamped.transform.translation.x = 1.0;
  static_transform_stamped.transform.translation.y = 1.0;
  static_transform_stamped.transform.translation.z = 0.0;
  static_transform_stamped.transform.rotation.w = 1.0;

  tf_broadcaster->sendTransform(static_transform_stamped);

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());

    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
