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

// #include <string>
// #include <utility>
// #include <limits>

#include "perception/IsDoorOpen.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
// #include "perception_system/PerceptionUtils.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsDoorOpen::IsDoorOpen(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf), door_threshold_(2.0)
{
  config().blackboard->get("node", node_);
  getInput("door_threshold", door_threshold_);
  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 100, std::bind(&IsDoorOpen::laser_callback, this, _1));
  // config().blackboard->get("cam_frame", cam_frame_);
}

BT::NodeStatus IsDoorOpen::tick()
{
  RCLCPP_INFO(node_->get_logger(), "IsDoorOpen ticked");
  rclcpp::spin_some(node_->get_node_base_interface());
  // if(door_open_)
  // {
  //   return BT::NodeStatus::SUCCESS;
  // }
  // else
  // {
  //   return BT::NodeStatus::FAILURE;
  // }

  // return BT::NodeStatus::SUCCESS; //test, change to SUCCESS
  if (last_scan_ == nullptr) {
    return BT::NodeStatus::FAILURE;
  }
  auto mid = last_scan_->ranges.size() / 2;
  for (size_t i = mid - 2; i <= mid + 2; i++) {
    if (last_scan_->ranges[i] > door_threshold_) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }
}

void IsDoorOpen::laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  // auto mid = msg->ranges.size() / 2;
  // for(size_t i = mid -2; i<= mid + 2; i++)
  // {
  //   if(msg->ranges[i] < door_threshold_)
  //   {
  //     door_open_ = true;
  //     return;
  //   }
  // }
  RCLCPP_INFO(node_->get_logger(), "Laser callback");
  last_scan_ = std::move(msg);
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsDoorOpen>("IsDoorOpen");
}
