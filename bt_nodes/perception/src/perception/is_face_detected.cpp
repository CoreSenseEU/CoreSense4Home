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

#include "perception/is_face_detected.hpp"

#include <limits>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "sensor_msgs/msg/image.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsFaceDetected::IsFaceDetected(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  max_depth_(std::numeric_limits<double>::max()),
  max_entities_(1)
{
  config().blackboard->get("node", node_);
  hri_listener_ = hri::HRIListener::create(node_);


  getInput("cam_frame", cam_frame_);
  getInput("max_entities", max_entities_);
  getInput("max_depth", max_depth_);

  hri_listener_->setReferenceFrame(cam_frame_);
}

BT::NodeStatus IsFaceDetected::tick()
{
  faces_ = hri_listener_->getFaces();
  
  if (faces_.size() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "[IsFaceDetected] No faces detected");
    return BT::NodeStatus::FAILURE;
  }

  // check for the distance :
  for (auto & face : faces_) {
    auto transform = face.second->gazeTransform();
    if (!transform.has_value()) {
      continue;
    }
    auto transform_value = transform.value();
    auto distance = std::hypot(transform_value.transform.translation.x,
                               transform_value.transform.translation.y,
                               transform_value.transform.translation.z);
    if (distance > max_depth_) {
      faces_.erase(face.first);
    }
  }

  if (faces_.size() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "[IsFaceDetected] No faces detected within the distance");
    return BT::NodeStatus::FAILURE;
  }

  // order the faces by distance
  ordered_faces_ = std::vector<std::pair<std::string, hri::FacePtr>>(faces_.begin(), faces_.end());
  std::sort(ordered_faces_.begin(), ordered_faces_.end(), [](auto & a, auto & b) {
    auto transform_a = a.second->gazeTransform().value();
    auto transform_b = b.second->gazeTransform().value();
    return std::hypot(transform_a.transform.translation.x,
                      transform_a.transform.translation.y,
                      transform_a.transform.translation.z) <
           std::hypot(transform_b.transform.translation.x,
                      transform_b.transform.translation.y,
                      transform_b.transform.translation.z);
  });

  // get the ID of the best detection
  setOutput("best_detection", ordered_faces_.front().first);
  auto i = 0;
  for (const auto& pair : ordered_faces_) {
      ids_.push_back(pair.first);
      if (++i >= max_entities_) {
        break;
      }
  }
  setOutput("faces", ids_);
  ids_.clear();
  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsFaceDetected>("IsFaceDetected");
}
