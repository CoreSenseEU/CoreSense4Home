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

#include "perception/filter_entity.hpp"

#include <limits>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

FilterEntity::FilterEntity(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  RCLCPP_DEBUG(node_->get_logger(), "FilterEntity initialized");
}

void FilterEntity::halt() {RCLCPP_INFO(node_->get_logger(), "FilterEntity halted");}

geometry_msgs::msg::TransformStamped FilterEntity::update_state_observer(
  const geometry_msgs::msg::TransformStamped & entity)
{
  filtered_entity_.transform.translation.x =
    filtered_entity_.transform.translation.x +
    lambda_ * (entity.transform.translation.x - filtered_entity_.transform.translation.x);
  filtered_entity_.transform.translation.y =
    filtered_entity_.transform.translation.y +
    lambda_ * (entity.transform.translation.y - filtered_entity_.transform.translation.y);
  filtered_entity_.transform.translation.z =
    filtered_entity_.transform.translation.z +
    lambda_ * (entity.transform.translation.z - filtered_entity_.transform.translation.z);
  filtered_entity_.header.stamp = entity.header.stamp;
  return filtered_entity_;
}
geometry_msgs::msg::TransformStamped FilterEntity::initialize_state_observer(
  const geometry_msgs::msg::TransformStamped & entity)
{
  filtered_entity_ = entity;
  filtered_entity_.child_frame_id = entity.child_frame_id + "_filtered";
  return filtered_entity_;
}

BT::NodeStatus FilterEntity::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_DEBUG(node_->get_logger(), "FilterEntity ticked");
    getInput("frame", frame_);
    getInput("lambda", lambda_);
    config().blackboard->get("tf_buffer", tf_buffer_);
    config().blackboard->get("tf_broadcaster", tf_broadcaster_);
  }
  RCLCPP_DEBUG(node_->get_logger(), "FilterEntity filtering frame %s", frame_.c_str());

  geometry_msgs::msg::TransformStamped entity_transform_now_msg;

  try {
    entity_transform_now_msg = tf_buffer_->lookupTransform("odom", frame_, tf2::TimePointZero);
    RCLCPP_INFO(
      node_->get_logger(), "Position %s to %s: %f %f %f", frame_.c_str(), "odom",
      entity_transform_now_msg.transform.translation.x,
      entity_transform_now_msg.transform.translation.y,
      entity_transform_now_msg.transform.translation.z);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform %s to %s: %s", frame_.c_str(), "odom", ex.what());
    RCLCPP_INFO(node_->get_logger(), "Cannot transform");

    return BT::NodeStatus::SUCCESS;
  }
  geometry_msgs::msg::TransformStamped filtered_entity;
  if (state_obs_initialized_) {
    filtered_entity = update_state_observer(entity_transform_now_msg);
  } else {
    filtered_entity = initialize_state_observer(entity_transform_now_msg);
    state_obs_initialized_ = true;
  }
  filtered_entity.child_frame_id = frame_ + "_filtered";
  tf_broadcaster_->sendTransform(filtered_entity);
  setOutput("filtered_frame", filtered_entity.child_frame_id);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::FilterEntity>("FilterEntity");
}
