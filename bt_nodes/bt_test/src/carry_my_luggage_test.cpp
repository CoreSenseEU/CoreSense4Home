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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  // options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    "carry_my_luggage", options);

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("init_carry_bt_node"));
  factory.registerFromPlugin(loader.getOSName("is_detected_bt_node"));
  factory.registerFromPlugin(loader.getOSName("is_pointing_bt_node"));
  factory.registerFromPlugin(loader.getOSName("move_to_bt_node"));
  factory.registerFromPlugin(loader.getOSName("move_to_predefined_bt_node"));
  factory.registerFromPlugin(loader.getOSName("look_at_bt_node"));
  factory.registerFromPlugin(loader.getOSName("speak_bt_node"));
  factory.registerFromPlugin(loader.getOSName("is_entity_moving_bt_node"));
  factory.registerFromPlugin(loader.getOSName("dialogConfirmation_bt_node"));


  std::string pkgpath = ament_index_cpp::get_package_share_directory("bt_test");
  std::string xml_file = pkgpath + "/bt_xml/carry_my_luggage.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 2666, 2667);

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
