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

#ifndef PERCEPTION__FILTER_OBJECT_HPP_
#define PERCEPTION__FILTER_OBJECT_HPP_

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "perception_system/PerceptionListener.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace perception {

struct ObjectInfo {
  std::string class_type;
  float size;
  float weight;
};

using namespace std::chrono_literals;

class FilterObject : public BT::ActionNodeBase {
public:
  explicit FilterObject(const std::string &xml_tag_name,
                        const BT::NodeConfiguration &conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts() {
    return BT::PortsList({
        BT::InputPort<std::vector<std::string>>("frames"),
        BT::InputPort<std::string>("size", "unknown", "size"),
        BT::InputPort<std::string>("weight", "unknown", "weight"),
        BT::InputPort<std::string>("class", "unknown", "class"),
        BT::InputPort<std::string>("cam_frame"),
        BT::OutputPort<std::string>("filtered_object"),
    });
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;

  std::vector<std::string>
  extractClassNames(const std::vector<std::string> &frames);
  std::string getObject(const std::vector<std::string> &frames,
                        std::function<bool(float, float)> compare,
                        float ObjectInfo::*info);
  int publicTF_map2object(
      const perception_system_interfaces::msg::Detection &detected_object);

  std::vector<std::string> frames_;
  std::string size_;
  std::string weight_;
  std::string class_;
  std::string camera_frame_;

  std::map<std::string, perception::ObjectInfo> objects_;

  std::string filtered_object_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

} // namespace perception

#endif // PERCEPTION__FILTER_OBJECT_HPP_