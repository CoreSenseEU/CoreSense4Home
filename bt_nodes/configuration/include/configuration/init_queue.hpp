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

#ifndef CONFIGURATION__INIT_QUEUE_HPP_
#define CONFIGURATION__INIT_QUEUE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/actions/pop_from_queue.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace configuration
{

class InitProtectedQueue : public BT::ActionNodeBase
{
public:
  explicit InitProtectedQueue(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("port"),
      });
  }

private:
  std::string port_;
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
};

}   // namespace configuration

#endif  // CONFIGURATION__INIT_QUEUE_HPP_
