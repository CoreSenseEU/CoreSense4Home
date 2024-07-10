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

#include "configuration/Deferred.hpp"

namespace configuration
{

Deferred::Deferred(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf) {}

void Deferred::halt() {}

BT::NodeStatus Deferred::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    bool by_content = true;
    bt_xml_ = getInput<std::string>("xml");
    if (!bt_xml_ || bt_xml_.value().empty()) {
      by_content = false;
      bt_pkg_ = getInput<std::string>("bt_pkg");
      if (!bt_pkg_ || bt_pkg_.value().empty()) {
        throw BT::RuntimeError("Missing required input [xml]/[bt_pkg]");
      }
      rel_path_ = getInput<std::string>("rel_path");
      if (!rel_path_ || rel_path_.value().empty()) {
        throw BT::RuntimeError("Missing required input [rel_path]");
      }
    }
    plugins_ = getInput<std::vector<std::string>>("plugins");

    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    if (plugins_) {
      for (const auto &plugin : plugins_.value()) {
        if (plugin == "ConsumeQueue") {
          factory.registerNodeType<BT::ConsumeQueue<geometry_msgs::msg::TransformStamped>>("ConsumeQueue");
        } else {
          factory.registerFromPlugin(loader.getOSName(plugin));
        }
      }
    } else {
      std::cerr << "[DeferredBT] WARNING: No plugins provided" << std::endl;
    }

    if (by_content) {
      auto node =
        config().blackboard->get<std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode>>(
        "node");
      RCLCPP_INFO(node->get_logger(), bt_xml_.value().c_str());

      subtree_ =
        factory.createTreeFromText(bt_xml_.value(), config().blackboard);
    } else {
      std::string xml_path =
        ament_index_cpp::get_package_share_directory(bt_pkg_.value()) + "/" +
        rel_path_.value();
      subtree_ = factory.createTreeFromFile(xml_path, config().blackboard);
    }
    // try {
    //   auto pub_blackboard = config().blackboard->get<std::shared_ptr<BT::PublisherZMQ>>("publisher_zmq");
    //   pub_blackboard.reset();
    // } catch (const std::exception &e) {
    //   std::cerr << "[DeferredBT] WARNING: No publisher_zmq found" << std::endl;
    // }

    // publisher_zmq_ = std::make_unique<BT::PublisherZMQ>(subtree_, 10, 2666, 2667);
  }

  auto state = subtree_.rootNode()->executeTick();

//  if (state == BT::NodeStatus::FAILURE || state == BT::NodeStatus::SUCCESS) {
//     publisher_zmq_.reset();
//   }
  return state;
}

}  // namespace configuration

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<configuration::Deferred>("Deferred");
}
