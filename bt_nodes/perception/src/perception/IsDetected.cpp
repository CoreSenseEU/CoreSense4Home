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

#include <string>
#include <utility>
#include <limits>

#include "perception/IsDetected.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "perception_system/PerceptionUtils.hpp"


namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsDetected::IsDetected(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  max_depth_(std::numeric_limits<double>::max()),
  max_entities_(1)
{
  config().blackboard->get("node", node_);

  getInput("interest", interest_);
  getInput("cam_frame", cam_frame_);
  getInput("confidence", threshold_);
  getInput("max_entities", max_entities_);
  getInput("order", order_);
  getInput("max_depth", max_depth_);
  getInput("person_id", person_id_);

  pl::getInstance()->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  pl::getInstance()->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

}

BT::NodeStatus
IsDetected::tick()
{
  pl::getInstance()->set_interest(interest_, true);
  pl::getInstance()->update(30);

  rclcpp::spin_some(pl::getInstance()->get_node_base_interface());

  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_DEBUG(node_->get_logger(), "IsDetected ticked");
    config().blackboard->get("tf_buffer", tf_buffer_);
    config().blackboard->get("tf_broadcaster", tf_broadcaster_);
  }

  auto detections = pl::getInstance()->get_by_type(interest_);

  if (detections.empty() ) {
    RCLCPP_INFO(node_->get_logger(), "No detections");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "Processing detections...");

  if (order_ == "color") {
    // sorted by the distance to the color person we should sort it by distance and also by left to right or right to left
    std::sort(
      detections.begin(), detections.end(),
      [this](const auto & a, const auto & b) {
        return perception_system::diffIDs(
          this->person_id_,
          a.color_person) <
        perception_system::diffIDs(this->person_id_, b.color_person);
      }
    );
  } else if (order_ == "depth") {
    std::sort(
      detections.begin(), detections.end(),
      [this](const auto & a, const auto & b) {
        return a.center3d.position.z < b.center3d.position.z;
      }
    );

  }
  RCLCPP_INFO(node_->get_logger(), "Detections sorted");
  // implement more sorting methods

  auto entity_counter = 0;
  for (auto it = detections.begin(); it != detections.end() && entity_counter < max_entities_; ) {
    auto const & detection = *it;

    if (detection.score <= threshold_ || detection.center3d.position.z > max_depth_) {
      it = detections.erase(it);
    } else {
      frames_.push_back(detection.class_name + "_" + std::to_string(entity_counter));
      if(publicTF_map2object(detection, detection.class_name + "_" + std::to_string(entity_counter)) == -1){
        return BT::NodeStatus::FAILURE;
      }
      ++it;
      ++entity_counter;
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Detections sorted and filtered");
  if (frames_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "No detections after filter");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("frames", frames_);
  frames_.clear();
  RCLCPP_INFO(node_->get_logger(), "Detections published");
  return BT::NodeStatus::SUCCESS; //test, change to SUCCESS
}

int
IsDetected::publicTF_map2object(
  const perception_system_interfaces::msg::Detection & detected_object,
  const std::string & frame_name)
{
  geometry_msgs::msg::TransformStamped map2camera_msg;
  try {
    map2camera_msg = tf_buffer_->lookupTransform(
      "map", cam_frame_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform %s to %s: %s",
      "map", cam_frame_.c_str(), ex.what());
    return -1;
  }
  tf2::Transform camera2object;
  camera2object.setOrigin(
    tf2::Vector3(
      detected_object.center3d.position.x,
      detected_object.center3d.position.y,
      detected_object.center3d.position.z));
  camera2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  tf2::Transform map2camera;
  tf2::fromMsg(map2camera_msg.transform, map2camera);

  tf2::Transform map2object = map2camera * camera2object;
  // create a transform message from tf2::Transform
  geometry_msgs::msg::TransformStamped map2object_msg;

  map2object_msg.header.stamp = node_->now();
  map2object_msg.header.frame_id = "map";
  map2object_msg.child_frame_id = frame_name;
  map2object_msg.transform = tf2::toMsg(map2object);

  tf_broadcaster_->sendTransform(map2object_msg);
  return 0;
}

}  // namespace perception


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<perception::IsDetected>("IsDetected");
}
