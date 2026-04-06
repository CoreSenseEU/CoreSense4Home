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

#ifndef DIALOG__EXPLAIN_HPP_
#define DIALOG__EXPLAIN_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "explainability_msgs/action/generate_explanation.hpp"
#include "hri/dialog/BTActionNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace dialog
{

/**
 * @class Explain
 * @brief BT action node that calls the cs4home-explainability GenerateExplanation
 *        action server and outputs the resulting explanation text to an output port.
 *
 * The explainability server reads /bt_status autonomously to identify the
 * cause of failure. The caller only needs to provide the node name (question)
 * and auto_triggered flag.
 *
 * A Speak node should be placed after this node in the BT to speak the result.
 *
 * Ports:
 *   Input  - question       : Name of the BT node whose failure is being explained
 *   Input  - auto_triggered : Always true; passed straight to the action server
 *   Output - explanation    : Natural-language explanation string from the server
 */
class Explain
  : public dialog::BtActionNode<
    explainability_msgs::action::GenerateExplanation,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit Explain(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("question", "Name of the failing BT node to explain"),
        BT::InputPort<bool>("auto_triggered", true, "Always true; server determines failure cause"),
        BT::OutputPort<std::string>("explanation", "Natural-language explanation from the server"),
      });
  }
};

}  // namespace dialog

#endif  // DIALOG__EXPLAIN_HPP_
