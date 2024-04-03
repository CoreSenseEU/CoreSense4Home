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
#include <vector>
#include <limits>

#include "perception/is_entity_moving.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "perception_system/PerceptionUtils.hpp"


namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsEntityMoving::IsEntityMoving(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  config().blackboard->get("cam_frame", cam_frame_);

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  //delte this :
  cam_frame_ = "head_front_camera_link_color_optical_frame";
}

BT::NodeStatus
IsEntityMoving::tick()
{
  getInput("frame", frame_);
  getInput("velocity_tolerance", velocity_tolerance_);
  getInput("max_iterations", max_iterations_);
  RCLCPP_INFO(node_->get_logger(), "IsEntityMoving ticked");

  geometry_msgs::msg::TransformStamped entity_transform_now_msg;

  try {
    entity_transform_now_msg = tf_buffer_->lookupTransform(
      "map",
      frame_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform %s to %s: %s",
      frame_.c_str(), "map", ex.what());
    return BT::NodeStatus::SUCCESS; // WE ASSUME IT IS MOVING FOR NOW
  }
  if (entity_transforms_.size() < max_iterations_) {
    entity_transforms_.push_back(entity_transform_now_msg);
    return BT::NodeStatus::SUCCESS;
  } else if (entity_transforms_.size() == max_iterations_) {
    for (auto it = entity_transforms_.begin(); it != entity_transforms_.end(); ++it) {
      auto velocity_x = ((std::next(it))->transform.translation.x - it->transform.translation.x) /
        (((std::next(it))->header.stamp.sec + (std::next(it))->header.stamp.nanosec * 1.0e-9 ) -
        (it->header.stamp.sec + it->header.stamp.nanosec * 1.0e-9));
      auto velocity_y = ((std::next(it))->transform.translation.y - it->transform.translation.y) /
        (((std::next(it))->header.stamp.sec + (std::next(it))->header.stamp.nanosec * 1.0e-9 ) -
        (it->header.stamp.sec + it->header.stamp.nanosec * 1.0e-9));

      velocities_.push_back(std::hypot(velocity_x, velocity_y));
    }
    auto vel = std::accumulate(velocities_.begin(), velocities_.end(), 0.0) / velocities_.size();
    RCLCPP_INFO(node_->get_logger(), "Velocity: %f", vel);

    if (vel > velocity_tolerance_) {
      RCLCPP_INFO(node_->get_logger(), "Entity is moving");
      return BT::NodeStatus::SUCCESS;
    }
    RCLCPP_INFO(node_->get_logger(), "Entity is not moving");
    velocities_.clear();
    entity_transforms_.clear();
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;


}


}  // namespace perception


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<perception::IsEntityMoving>("IsEntityMoving");
}
