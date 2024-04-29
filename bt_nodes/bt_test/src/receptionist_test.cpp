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





int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  class ReceptionistNode : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
  {
  public:
    ReceptionistNode(const rclcpp::NodeOptions & options):CascadeLifecycleNode("receptionist", options)
    {};
  };
  rclcpp::NodeOptions options;
  // options.use_intra_process_comms(true);

  // options.automatically_declare_parameters_from_overrides(true);
  

  // auto node = rclcpp::Node::make_shared("receptionist", options);
  auto node = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>("receptionist",options);
  // node->set_parameter(rclcpp::Parameter("allow_duplicate_names", false));

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("set_wp_bt_node"));
  factory.registerFromPlugin(loader.getOSName("init_receptionist_bt_node"));
  factory.registerFromPlugin(loader.getOSName("move_to_bt_node"));
  factory.registerFromPlugin(loader.getOSName("speak_bt_node"));
  factory.registerFromPlugin(loader.getOSName("is_detected_bt_node"));
  factory.registerFromPlugin(loader.getOSName("store_detection_bt_node"));
  factory.registerFromPlugin(loader.getOSName("look_at_bt_node"));
  factory.registerFromPlugin(loader.getOSName("listen_bt_node"));
  factory.registerFromPlugin(loader.getOSName("query_bt_node"));
  factory.registerFromPlugin(loader.getOSName("dialogConfirmation_bt_node"));
  factory.registerFromPlugin(loader.getOSName("pan_bt_node"));
  factory.registerFromPlugin(loader.getOSName("is_sittable_bt_node"));
  factory.registerFromPlugin(loader.getOSName("point_at_bt_node"));
  factory.registerFromPlugin(loader.getOSName("move_to_predefined_bt_node"));
  factory.registerFromPlugin(loader.getOSName("add_guest_to_count_bt_node"));
  factory.registerFromPlugin(loader.getOSName("filter_entity_bt_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("bt_test");
  std::string xml_file = pkgpath + "/bt_xml/receptionist.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 2666, 2667);

  rclcpp::Rate rate(30);

  bool finish = false;
  // std::thread queue_thread([&]() {
  //   rclcpp::spin(node->get_node_base_interface());
  // });

  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;
    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }
  // queue_thread.join();

  rclcpp::shutdown();
  return 0;
}
