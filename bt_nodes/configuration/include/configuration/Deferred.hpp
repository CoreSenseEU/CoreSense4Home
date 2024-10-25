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

#ifndef CONFIGURATION__DEFERRED_HPP_
#define CONFIGURATION__DEFERRED_HPP_


#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/actions/pop_from_queue.hpp"
#include "configuration/ConsumeQueueWithFailure.h"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace configuration
{

class Deferred : public BT::ActionNodeBase
{
public:
  explicit Deferred(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("bt_pkg"), // package where the XML is located
        BT::InputPort<std::string>("rel_path"), // relative path to the XML
        BT::InputPort<std::string>("xml"), // XML corresponding to the BT to be executed
        BT::InputPort<std::vector<std::string>>("plugins"), // plugins to load
      });
  }

private:
  BT::Optional<std::string> bt_xml_, bt_pkg_, rel_path_;
  BT::Optional<std::vector<std::string>> plugins_;
  BT::Tree subtree_;
  std::unique_ptr<BT::PublisherZMQ> publisher_zmq_;
};

}   // namespace configuration

#endif  // DEFERRED__HPP_
