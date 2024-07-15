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

#ifndef PERCEPTION__FILTER_PREV_DETECTIONS_HPP_
#define PERCEPTION__FILTER_PREV_DETECTIONS_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <string>
#include <list>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/actions/pop_from_queue.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace perception
{

using namespace std::chrono_literals;

class FilterPrevDetections : public BT::ActionNodeBase
{
public:
  explicit FilterPrevDetections(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::shared_ptr<BT::ProtectedQueue<geometry_msgs::msg::TransformStamped>>>(
          "prev_detections"),
        BT::InputPort<std::vector<std::string>>("new_detections"),
        BT::InputPort<double>("margin"),
        BT::InputPort<std::string>("frame_id"),
      });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;

  std::shared_ptr<BT::ProtectedQueue<geometry_msgs::msg::TransformStamped>> prev_detections_;
  std::vector<std::string> new_detections_;
  double margin_;
  std::string frame_id_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace perception

#endif  // PERCEPTION__FILTER_PREV_DETECTIONS_HPP_
