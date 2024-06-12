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

#ifndef HRI__LISTEN_HPP_
#define HRI__LISTEN_HPP_

#include <algorithm>
#include <cstdint>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "hri/dialog/BTActionNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "std_msgs/msg/int8.hpp"
#include "whisper_msgs/action/stt.hpp"

namespace dialog
{

class Listen : public BT::ActionNodeBase
{
public:
  explicit Listen(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({BT::OutputPort<std::string>("listen_text")});
  }

private:
  BT::NodeStatus on_idle();
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int8>::SharedPtr publisher_start_;
  std::shared_ptr<rclcpp_action::Client<whisper_msgs::action::STT>> client_;
  bool is_goal_sent_ = false;
  std::string text_ = "";
};

}  // namespace dialog

#endif  // HRI__LISTEN_HPP_
