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

#ifndef HRI__GEST_GUEST_INFO_HPP_
#define HRI__GEST_GUEST_INFO_HPP_

#include <chrono>
#include <functional>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/string.hpp"
#include "kb_msgs/srv/query.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "hri/bt_service_node.hpp"

namespace dialog
{

class GetGuestInfo : public hri::BtServiceNode<
    kb_msgs::srv::Query,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit GetGuestInfo(const std::string & xml_tag_name, const std::string & srv_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  void on_result() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<bool>("guest_attending", true, "Is the guest being attended?"),
        BT::InputPort<std::string>("guest_attending_id"),
        BT::OutputPort<std::string>("guest_name"),
        BT::OutputPort<std::string>("guest_drink"),
        BT::OutputPort<std::string>("guest_description"),
      });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr kb_publisher_;

  std::string guest_name_, guest_drink_, guest_description_, guest_attending_id_, guest_id_;
  bool guest_attending_;
};

}  // namespace dialog

#endif  // HRI__GEST_GUEST_INFO_HPP_