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

#ifndef robocup_hri__COMMAND_PLANNING_HPP_
#define robocup_hri__COMMAND_PLANNING_HPP_

#include <algorithm>
#include <string>
#include <vector>
#include <sstream>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "gpsr_msgs/srv/generate_plan.hpp"
#include "robocup_hri/bt_service_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "std_msgs/msg/int8.hpp"

namespace robocup_hri
{

class CommandPlanning
  : public robocup_hri::BtServiceNode<gpsr_msgs::srv::GeneratePlan,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit CommandPlanning(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  void on_result() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<std::string>("command"),
        BT::OutputPort<std::string>("actions"),
        BT::OutputPort<std::string>("bt_value")});
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  std::string command_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_start_;
};

} // namespace robocup_hri

#endif // robocup_hri__COMMAND_PLANNING_HPP_
