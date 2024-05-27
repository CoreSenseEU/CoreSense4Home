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

  auto node = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("behaviors_main");

  std::vector<std::string> plugins;
  std::string bt_xml_file;
  node->declare_parameter("plugins", plugins);
  node->declare_parameter("bt_xml_file", bt_xml_file);
  node->get_parameter("plugins", plugins);
  node->get_parameter("bt_xml_file", bt_xml_file);

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  for (const auto & plugin : plugins) {
    RCLCPP_INFO(node->get_logger(), "Loading BT Node: [%s]", plugin.c_str());
    factory.registerFromPlugin(loader.getOSName(plugin));
  }

  std::string pkgpath = ament_index_cpp::get_package_share_directory("robocup_bringup");
  std::string xml_file = pkgpath + "/bt_xml/" + bt_xml_file;

  RCLCPP_INFO(node->get_logger(), "Loading BT: [%s]", xml_file.c_str());

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.tickRoot() != BT::NodeStatus::RUNNING;
    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
