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

#ifndef PERCEPTION__ISDETECTED_HPP_
#define PERCEPTION__ISDETECTED_HPP_

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <string>
#include <map>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "perception_system/PerceptionListener.hpp"
#include "perception_system/PerceptionUtils.hpp"
#include "perception_system_interfaces/msg/detection_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

namespace perception
{

using pl = perception_system::PerceptionListener;

class IsDetected : public BT::ConditionNode
{
public:
  explicit IsDetected(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<int>("max_entities"), BT::InputPort<std::int64_t>("person_id"),
        BT::InputPort<std::string>("cam_frame"), BT::InputPort<std::string>("interest"),
        BT::InputPort<float>("confidence"),
        BT::InputPort<std::string>("order"), // todo: enum map or string?
        BT::InputPort<double>("max_depth"),
        BT::InputPort<std::string>("color", "unknown", "color"),
        BT::InputPort<std::string>("gesture", "unknown", "gesture"),
        BT::InputPort<std::string>("pose", "unknown", "pose"),
        BT::InputPort<bool>("pub_bb_img"),

        BT::OutputPort<std::vector<std::string>>("frames"),
        BT::OutputPort<std::string>("best_detection")});
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string interest_, order_, cam_frame_;
  double threshold_, max_depth_;
  int max_entities_;
  std::int64_t person_id_;
  std::vector<std::string> frames_;
  std::string color_;
  std::string gesture_;
  std::string pose_;

  double hue_threshold_{20.0};
  double saturation_threshold_{50.0};
  double value_threshold_{50.0};

  std::map<std::string, cv::Scalar> colors_;
  std::map<std::string, std::vector<int>> gestures_;
  std::map<int, std::string> pose_names_;

  bool pub_bb_img_{false};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bb_img_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  cv::Mat last_image_;
};

}  // namespace perception

#endif  // PERCEPTION__ISDETECTED_HPP_
