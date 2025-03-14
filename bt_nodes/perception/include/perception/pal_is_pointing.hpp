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

#ifndef PERCEPTION__PAL_IS_POINTING_HPP_
#define PERCEPTION__PAL_IS_POINTING_HPP_

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "hri_msgs/msg/ids_list.hpp"
#include "hri_msgs/msg/skeleton2_d.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "perception_system/PerceptionListener.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors.hpp>
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace perception
{

class PalIsPointing : public BT::ConditionNode
{
public:
  explicit PalIsPointing(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();
  ~PalIsPointing() override;
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<std::int64_t>("person_id"),
        BT::InputPort<std::string>("cam_frame"),
        BT::InputPort<int>("low_pointing_limit"),
        BT::InputPort<int>("high_pointing_limit"),
        BT::OutputPort<std::string>("output_frame")
      });
  }

private:

  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  rclcpp::Subscription<hri_msgs::msg::IdsList>::SharedPtr ids_list_sub_;

  hri_msgs::msg::IdsList::SharedPtr last_ids_list_msg_;

  std::string camera_frame_, current_id_, output_frame_, best_detection_so_far_, suffix_{"bag"};

  std::int64_t person_id_;
  int low_pointing_limit_, high_pointing_limit_;
  bool subs_created_{false}, update_subs_{false};
  geometry_msgs::msg::TransformStamped person_pose_;
  rclcpp::Time last_pose_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

}  // namespace perception

#endif  // PERCEPTION__PAL_IS_POINTING_HPP_
