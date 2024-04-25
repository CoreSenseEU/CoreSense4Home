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

#ifndef PERCEPTION__IS_SITTABLE_HPP_
#define PERCEPTION__IS_SITTABLE_HPP_

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "perception_system/PerceptionListener.hpp"
#include "rclcpp/rclcpp.hpp"

namespace perception
{

class IsSittable : public BT::ConditionNode
{
public:
  explicit IsSittable(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<std::string>("cam_frame")
       });
  }

private:

  rclcpp::Node::SharedPtr node_;

  std::string camera_frame_, chair_frame_;
  std::int64_t person_id_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;


  std::vector<std::string> sit_objects_ = {"chair", "couch", "bench", "sofa"};

  bool is_place_to_sit_;
  std::string place_to_sit_;
  bool is_person_;
  float threshold_{0.65};

  bool 
  check_object_class(const std::string & obj, const std::vector<perception_system_interfaces::msg::Detection> & msg);

  bool 
  check_free_space(const geometry_msgs::msg::Point & bb1, const geometry_msgs::msg::Point & bb2 );
  
  geometry_msgs::msg::Point 
  retrieve_bb(const std::string & obj, const std::vector<perception_system_interfaces::msg::Detection> & msg);

};

}  // namespace perception

#endif  // PERCEPTION__IS_SITTABLE_HPP_
