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

#ifndef HEAD__PAN_HPP_
#define HEAD__PAN_HPP_


#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ctrl_support/BTActionNode.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace head
{

class Pan : public motion::BtActionNode<
  control_msgs::action::FollowJointTrajectory, rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit Pan(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);


  void on_tick() override;
  void on_feedback() override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("tf_frame"),
      });
  }
private:
  rclcpp::Node::SharedPtr node_;
  BT::Optional<std::string> point_to_pan_;
};

}  // namespace receptionist

#endif  // RECEPTIONIST__BEHAVIOR_TREES_NODES__PAN_TO_POINT_HPP_
