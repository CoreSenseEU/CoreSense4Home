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

#ifndef PERCEPTION__IS_FACE_DETECTED_HPP_
#define PERCEPTION__IS_FACE_DETECTED_HPP_

// #include <string>
// #include <algorithm>

#include <tf2/transform_datatypes.h>


#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hri/hri.hpp"
#include "hri/face.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"


namespace perception
{


class IsFaceDetected : public BT::ConditionNode
{
public:
  explicit IsFaceDetected(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {BT::InputPort<int>("max_entities"),
       BT::InputPort<std::string>("cam_frame"),
       BT::InputPort<double>("max_depth"),
       BT::OutputPort<std::string>("best_detection"),
       BT::OutputPort<std::vector<std::string>>("faces")});
  }

private:
  std::string cam_frame_;
  double max_depth_;
  int max_entities_;
  std::int64_t person_id_;
  std::map<hri::ID,hri::FacePtr> faces_;
  std::vector<std::pair<hri::ID, hri::FacePtr>> ordered_faces_;
  std::vector<std::string> ids_;

  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  std::shared_ptr<hri::HRIListener> hri_listener_;


};


}  // namespace perception

#endif  // PERCEPTION__IS_FACE_DETECTED_HPP_
