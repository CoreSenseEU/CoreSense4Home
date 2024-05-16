#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    "ask_test_node");
  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("speak_bt_node"));
  factory.registerFromPlugin(loader.getOSName("listen_bt_node"));
  factory.registerFromPlugin(loader.getOSName("query_bt_node"));
  factory.registerFromPlugin(loader.getOSName("dialogConfirmation_bt_node"));
  std::string pkgpath = ament_index_cpp::get_package_share_directory("bt_test");
  std::string xml_file = pkgpath + "/bt_xml/ask_test.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 2666, 2667);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::Rate rate(10);

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  bool finish = false;
  while (!finish && rclcpp::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());

    status = tree.rootNode()->executeTick();
    finish = (status == BT::NodeStatus::SUCCESS) ||
      (status == BT::NodeStatus::FAILURE);

    rate.sleep();
  }
  std::cout << "Ask Test Finished with status: " << status << std::endl;
  rclcpp::shutdown();
  return 0;
}
