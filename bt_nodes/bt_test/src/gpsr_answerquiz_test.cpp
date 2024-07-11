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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  // options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    "gpsr_answerquiz_test", options);

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("deferred_bt_node"));
  factory.registerFromPlugin(loader.getOSName("setup_gpsr_bt_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("bt_test");
  std::string xml_file = pkgpath + "/bt_xml/gpsr_answerquiz_test.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("text_value", "What day is tomorrow?");

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  // auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 2666, 2667);
  // blackboard->set("publisher_zmq", publisher_zmq);

  node->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::Rate rate(30);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());

    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
