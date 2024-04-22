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

#ifndef NAVIGATION__LOOK_AT_HPP_
#define NAVIGATION__LOOK_AT_HPP_

#include <tf2_ros/buffer.h>

#include <memory>
#include <string>

#include "attention_system_msgs/msg/attention_command.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace head
{

class LookAt : public BT::ActionNodeBase
{
public:
  explicit LookAt(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<std::vector<std::string>>("tf_frames"),
        BT::InputPort<std::string>("tf_frame")});
  }

private:
  rclcpp::Node::SharedPtr node_;
  geometry_msgs::msg::PoseStamped pose_;
  rclcpp::Publisher<attention_system_msgs::msg::AttentionCommand>::SharedPtr attention_points_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr test_pub_;
  std::vector<std::string> tf_frames_;
  std::string tf_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace navigation

#endif  // NAVIGATION__LOOK_AT_HPP_