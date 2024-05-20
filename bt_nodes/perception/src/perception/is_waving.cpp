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

#include "perception/is_waving.hpp"

#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "perception_system/PerceptionUtils.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

using pl = perception_system::PerceptionListener;

IsWaving::IsWaving(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  getInput("cam_frame", camera_frame_);
}

// Distance between two transformations
double tfs_distance(
  const geometry_msgs::msg::TransformStamped & tf1,
  const geometry_msgs::msg::TransformStamped & tf2)
{
  double dist = sqrt(
    pow(tf1.transform.translation.x - tf2.transform.translation.x, 2) +
    pow(tf1.transform.translation.y - tf2.transform.translation.y, 2) +
    pow(tf1.transform.translation.z - tf2.transform.translation.z, 2));

  return dist;
}

// Mean of two transformations
geometry_msgs::msg::TransformStamped tfs_mean(
  const geometry_msgs::msg::TransformStamped & tf1,
  const geometry_msgs::msg::TransformStamped & tf2)
{
  geometry_msgs::msg::TransformStamped mean = tf1;

  mean.transform.translation.x = (tf1.transform.translation.x + tf2.transform.translation.x) / 2;
  mean.transform.translation.y = (tf1.transform.translation.y + tf2.transform.translation.y) / 2;
  mean.transform.translation.z = (tf1.transform.translation.z + tf2.transform.translation.z) / 2;

  return mean;
}

bool
IsWaving::publish_tf_customer(
  const perception_system_interfaces::msg::Detection & detected_customer)
{
  geometry_msgs::msg::TransformStamped map2camera_msg;

  try {
    map2camera_msg = tf_buffer_->lookupTransform("map", camera_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "[IsWaving] Could not transform %s to %s: %s", "map",
      camera_frame_.c_str(), ex.what());
    return false;
  }

  tf2::Transform camera2person;
  camera2person.setOrigin(
    tf2::Vector3(
      detected_customer.center3d.position.x, detected_customer.center3d.position.y,
      detected_customer.center3d.position.z));
  camera2person.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  tf2::Transform map2camera;
  tf2::fromMsg(map2camera_msg.transform, map2camera);

  tf2::Transform map2person = map2camera * camera2person;

  geometry_msgs::msg::TransformStamped map2person_msg;
  map2person_msg.header.frame_id = "map";
  map2person_msg.child_frame_id = "customer";

  map2person_msg.transform = tf2::toMsg(map2person);
  map2person_msg.transform.translation.z = 0.0;

    // if person_pose_ is not initialized, initialize it
  if (person_pose_.header.frame_id.empty()) {
    person_pose_ = map2person_msg;
    last_pose_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();
  } else {
    auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
    auto diff = now - last_pose_time_;
    if (diff.seconds() > 0.5) {
      person_pose_ = map2person_msg;
      last_pose_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();
    } else {
      // calculate the distance between the current position and the new position
      double distance = tfs_distance(person_pose_, map2person_msg);
      // if the distance is greater than a threshold, update the position
      if (distance < 0.5) {
        geometry_msgs::msg::TransformStamped mean = tfs_mean(person_pose_, map2person_msg);
        person_pose_ = mean;
        last_pose_ = rclcpp::Clock(RCL_STEADY_TIME).now();
      }
    }
  }
  RCLCPP_INFO(node_->get_logger(), "[IsWaving] Bag direction %s", bag_frame_.c_str());

  tf_static_broadcaster_->sendTransform(person_pose_);
  return true;
}

bool
IsWaving::is_person_waving(
  const perception_system_interfaces::msg::Detection & detected_person)
{
  return detected_object.pointing_direction >= 5;
}

BT::NodeStatus IsWaving::tick()
{

  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_DEBUG(node_->get_logger(), "IsWaving ticked");
    config().blackboard->get("tf_buffer", tf_buffer_);
    config().blackboard->get("tf_static_broadcaster", tf_static_broadcaster_);
  }

  pl::getInstance(node_)->set_interest("person", true);
  pl::getInstance(node_)->update(true);

  std::vector<perception_system_interfaces::msg::Detection> detections;
  detections = pl::getInstance(node_)->get_by_type("person");

  if (detections.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  for (auto & detected_person : detections) {
    if (is_person_waving(detected_person)) {
      if (!publish_tf_customer(detected_person))
        return BT::NodeStatus::FAILURE;
      setOutput("customer_frame", "customer");
      return BT::NodeStatus::SUCCESS;
    }
  }

}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsWaving>("IsWaving");
}
