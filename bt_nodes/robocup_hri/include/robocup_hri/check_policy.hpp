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

#ifndef robocup_hri__CHECK_POLICY_HPP_
#define robocup_hri__CHECK_POLICY_HPP_

#include <algorithm>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "robocup_hri/dialog/BTActionNode.hpp"
#include "llama_msgs/action/generate_response.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "std_msgs/msg/int8.hpp"

namespace dialog
{

class CheckPolicy
  : public dialog::BtActionNode<
    llama_msgs::action::GenerateResponse, rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit CheckPolicy(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<std::string>("question"),
        BT::InputPort<std::string>("image_topic"),
        BT::OutputPort<bool>("output"),
        BT::OutputPort<std::string>("output_text")});
  }

private:
  std::string question_;
  std::string image_topic_;
  bool value_;
  sensor_msgs::msg::Image::SharedPtr image_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

};

}  // namespace dialog

#endif  // robocup_hri__CheckPolicy_HPP_
