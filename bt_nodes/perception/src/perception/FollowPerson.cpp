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

#include "perception/FollowPerson.hpp"

#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "perception_system/PerceptionListener.hpp"
#include "perception_system/PerceptionUtils.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

using pl = perception_system::PerceptionListener;

FollowPerson::FollowPerson(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  // config().blackboard->get("perception_listener", perception_listener_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  getInput("person_id", person_id_);
  getInput("camera_link", camera_link_);
}

void FollowPerson::halt() {RCLCPP_INFO(node_->get_logger(), "FollowPerson halted");}

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

int FollowPerson::publicTF_map2object(
  const perception_system_interfaces::msg::Detection & detected_object)
{
  geometry_msgs::msg::TransformStamped map2camera_msg;
  try {
    map2camera_msg = tf_buffer_->lookupTransform("map", camera_link_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform %s to %s: %s", "map", camera_link_.c_str(),
      ex.what());
    return -1;
  }

  tf2::Transform camera2object;
  camera2object.setOrigin(
    tf2::Vector3(
      detected_object.center3d.position.x, detected_object.center3d.position.y,
      detected_object.center3d.position.z));
  camera2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  tf2::Transform map2camera;
  tf2::fromMsg(map2camera_msg.transform, map2camera);

  tf2::Transform map2object = map2camera * camera2object;
  // create a transform message from tf2::Transform
  geometry_msgs::msg::TransformStamped map2object_msg;
  map2object_msg.header.stamp = detected_object.header.stamp;
  map2object_msg.header.frame_id = "map";
  map2object_msg.child_frame_id = "follow_person";
  map2object_msg.transform = tf2::toMsg(map2object);

  // if person_pose_ is not initialized, initialize it
  if (person_pose_.header.frame_id.empty()) {
    person_pose_ = map2object_msg;
    last_pose_ = rclcpp::Clock(RCL_STEADY_TIME).now();
  } else {
    auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
    auto diff = now - last_pose_;
    if (diff.seconds() > 0.5) {
      person_pose_ = map2object_msg;
      last_pose_ = rclcpp::Clock(RCL_STEADY_TIME).now();
    } else {
      // calculate the distance between the current position and the new position
      double distance = tfs_distance(person_pose_, map2object_msg);
      // if the distance is greater than a threshold, update the position
      if (distance < 0.5) {
        geometry_msgs::msg::TransformStamped mean = tfs_mean(person_pose_, map2object_msg);
        person_pose_ = mean;
        last_pose_ = rclcpp::Clock(RCL_STEADY_TIME).now();
      }
    }
  }

  tf_broadcaster_->sendTransform(person_pose_);
  return 0;
}

BT::NodeStatus FollowPerson::tick()
{
  pl::getInstance(node_)->set_interest("person", true);
  // pl::getInstance(node_)->set_interest("chair", true);
  pl::getInstance(node_)->update(30);
  pl::getInstance(node_)->publicTFinterest();

  std::vector<perception_system_interfaces::msg::Detection> detections;
  detections = pl::getInstance(node_)->get_by_type("person");

  if (detections.empty()) {
    // RCLCPP_INFO(node_->get_logger(), "No detections");
    return BT::NodeStatus::FAILURE;
  }

  perception_system_interfaces::msg::Detection best_detection;

  best_detection = detections[0];
  // TO-DO: Implement the best detection
  float best_detection_diff = perception_system::diffIDs(person_id_, best_detection.color_person);

  for (auto & detected_object : detections) {
    // Get the bounding box
    float min_diff = perception_system::diffIDs(person_id_, detected_object.color_person);

    if (min_diff < best_detection_diff) {
      // Display the results
      best_detection = detected_object;
      best_detection_diff = min_diff;
    }
  }

  RCLCPP_INFO(
    node_->get_logger(), "Best detection: %s, color_person: %ld, pointing: %d",
    best_detection.unique_id.c_str(), best_detection.color_person,
    best_detection.pointing_direction);

  int dev = publicTF_map2object(best_detection);
  if (dev != 0) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::FollowPerson>("FollowPerson");
}
