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
// See the License for the specific language governing permissions andGO2OBJECT
// limitations under the License.

#ifndef HRI__COMMAND_PLANNING_HPP_
#define HRI__COMMAND_PLANNING_HPP_

#include <algorithm>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "gpsr_msgs/srv/generate_plan.hpp"
#include "hri/dialog/BTActionNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "std_msgs/msg/int8.hpp"

namespace hri {

class CommandPlanning : public BT::ActionNodeBase {
public:
  explicit CommandPlanning(const std::string &xml_tag_name,
                           const BT::NodeConfiguration &conf);

  void halt();
  BT::NodeStatus tick();
  static BT::PortsList providedPorts() {
    return BT::PortsList({BT::InputPort<std::string>("command"),
                          BT::OutputPort<std::string>("actions"),
                          BT::OutputPort<std::string>("bt_value")});
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  rclcpp::Client<gpsr_msgs::srv::GeneratePlan>::SharedPtr generate_plan_client_;
  std::string command_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_start_;
};

} // namespace hri

#endif // HRI__COMMAND_PLANNING_HPP_
