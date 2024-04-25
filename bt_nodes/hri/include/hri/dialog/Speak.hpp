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

#ifndef DIALOG__SPEAK_HPP_
#define DIALOG__SPEAK_HPP_

#include <algorithm>
#include <cstdint>
#include <string>

#include "audio_common_msgs/action/tts.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "hri/dialog/BTActionNode.hpp"

#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"


namespace dialog
{

class Speak : public dialog::BtActionNode<
  audio_common_msgs::action::TTS, rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit Speak(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<std::string>("say_text"),
        BT::InputPort<std::string>("param")});
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int8>::SharedPtr publisher_start_;

  // rclcpp::Node::SharedPtr node_;
  //  rclcpp::ActionClient<audio_common_msgs::action::TTS>::SharedPtr
  //  tts_action_;

  // std::string text_;
};

} // namespace dialog

#endif // HRI__SPEAK_HPP_
