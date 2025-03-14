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

#include "perception/pal_is_pointing.hpp"

#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "perception_system/PerceptionUtils.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

using pl = perception_system::PerceptionListener;

PalIsPointing::PalIsPointing(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  ids_list_sub_ = node_->create_subscription<hri_msgs::msg::IdsList>(
    "/humans/bodies/tracked", 10,
    [this](const hri_msgs::msg::IdsList::SharedPtr msg) {
      RCLCPP_INFO_ONCE(node_->get_logger(), "IDs list received");
      last_ids_list_msg_ = msg;
    });
}
PalIsPointing::~PalIsPointing()
{
  RCLCPP_INFO(node_->get_logger(), "PalIsPointing destructor called");
}


BT::NodeStatus PalIsPointing::tick()
{
  rclcpp::spin_some(node_->get_node_base_interface());
  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_INFO(node_->get_logger(), "PalIsPointing ticked");
    config().blackboard->get("tf_buffer", tf_buffer_);
    config().blackboard->get("tf_static_broadcaster", tf_static_broadcaster_);
  }

  if (!last_ids_list_msg_ || last_ids_list_msg_->ids.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No bodies detected so far");
    return BT::NodeStatus::FAILURE;
  }

  std::string first_id = last_ids_list_msg_->ids[0];

  geometry_msgs::msg::TransformStamped r_hip_to_wrist_msg;
  geometry_msgs::msg::TransformStamped l_hip_to_wrist_msg;
  geometry_msgs::msg::TransformStamped torso_to_r_wrist_msg;
  geometry_msgs::msg::TransformStamped torso_to_l_wrist_msg;
  geometry_msgs::msg::TransformStamped map_to_person_msg;
  
  try {
    r_hip_to_wrist_msg = tf_buffer_->lookupTransform("waist_"+first_id, "r_wrist_"+first_id, tf2::TimePointZero);
    l_hip_to_wrist_msg = tf_buffer_->lookupTransform("waist_"+first_id, "l_wrist_"+first_id, tf2::TimePointZero);
    torso_to_r_wrist_msg = tf_buffer_->lookupTransform("torso_"+first_id, "r_wrist_"+first_id, tf2::TimePointZero);
    torso_to_l_wrist_msg = tf_buffer_->lookupTransform("torso_"+first_id, "l_wrist_"+first_id, tf2::TimePointZero);
    map_to_person_msg = tf_buffer_->lookupTransform("map", "body_"+first_id, tf2::TimePointZero);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "[PalIsPointing] Could not transform");
    return BT::NodeStatus::FAILURE;
  }

  auto distance_hip_wrist_r = sqrt(
    pow(r_hip_to_wrist_msg.transform.translation.x, 2) +
    pow(r_hip_to_wrist_msg.transform.translation.y, 2) +
    pow(r_hip_to_wrist_msg.transform.translation.z, 2));
  auto distance_hip_wrist_l = sqrt(
    pow(l_hip_to_wrist_msg.transform.translation.x, 2) +
    pow(l_hip_to_wrist_msg.transform.translation.y, 2) +
    pow(l_hip_to_wrist_msg.transform.translation.z, 2));  
  
  if (torso_to_r_wrist_msg.transform.translation.z > 0.0 || torso_to_l_wrist_msg.transform.translation.z > 0.0) {
    RCLCPP_INFO(node_->get_logger(), "Person is pointing up, invalid direction");
    return BT::NodeStatus::FAILURE;
  } 
  geometry_msgs::msg::TransformStamped bag_tf;
  bag_tf = map_to_person_msg;
  bag_tf.header.frame_id = "map";
  if (distance_hip_wrist_r > distance_hip_wrist_l) {
    bag_tf.transform.translation.x += 0.4;
    bag_tf.transform.translation.z = 0.0;
    tf_static_broadcaster_->sendTransform(bag_tf);
    bag_tf.child_frame_id = "left_bag";
    setOutput("output_frame", "left_bag");
  } else {
    bag_tf.transform.translation.x -= 0.4;
    bag_tf.transform.translation.z = 0.0;
    bag_tf.child_frame_id = "left_bag";
    setOutput("output_frame", "right_bag");
    tf_static_broadcaster_->sendTransform(bag_tf);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::PalIsPointing>("PalIsPointing");
}
