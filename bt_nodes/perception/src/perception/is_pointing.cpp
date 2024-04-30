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

#include "perception/is_pointing.hpp"

#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "perception_system/PerceptionUtils.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

using pl = perception_system::PerceptionListener;

IsPointing::IsPointing(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  // config().blackboard->get("perception_listener", perception_listener_);

  // tf_buffer_ =
  //   std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  // tf_listener_ =
  //   std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

  // pl::getInstance()->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  // pl::getInstance()->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

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

int IsPointing::publicTF_map2object(
  const perception_system_interfaces::msg::Detection & detected_object)
{
  geometry_msgs::msg::TransformStamped map2camera_msg;
  perception_system_interfaces::msg::Detection modified_detection;
  modified_detection = detected_object;

  try {
    map2camera_msg = tf_buffer_->lookupTransform("map", camera_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "[IsPointing] Could not transform %s to %s: %s", "map",
      camera_frame_.c_str(), ex.what());
    return -1;
  }

  // 0 is right, 1 is down-right, 2 is down, 3 is down-left, 4 is left, 5 is up-left, 6 is up, 7 is up-right
  if (detected_object.pointing_direction == 1) {
    bag_frame_ = "right_bag";
    modified_detection.center3d.position.x += 0.4;  // + or - ?
  } else if (detected_object.pointing_direction == 3) {
    modified_detection.center3d.position.x -= 0.4;  // + or - ?
    bag_frame_ = "left_bag";
  } 
  // else if (detected_object.pointing_direction == 2) {
  //   bag_frame_ = "center_bag";
  //   modified_detection.center3d.position.z -= 0.4;  // + or - ?
  // }
   else {
    return -1;
  }

  tf2::Transform camera2object;
  camera2object.setOrigin(
    tf2::Vector3(
      modified_detection.center3d.position.x, modified_detection.center3d.position.y,
      modified_detection.center3d.position.z));
  camera2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  tf2::Transform map2camera;
  tf2::fromMsg(map2camera_msg.transform, map2camera);

  tf2::Transform map2object = map2camera * camera2object;
  // create a transform message from tf2::Transform
  geometry_msgs::msg::TransformStamped map2object_msg;
  map2object_msg.header.frame_id = "map";
  map2object_msg.child_frame_id = bag_frame_;

  map2object_msg.transform = tf2::toMsg(map2object);
  map2object_msg.transform.translation.z = 0.0;

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
  RCLCPP_INFO(node_->get_logger(), "[IsPointing] Bag direction %s", bag_frame_.c_str());

  tf_static_broadcaster_->sendTransform(person_pose_);
  return 0;
}

BT::NodeStatus IsPointing::tick()
{
  pl::getInstance()->set_interest("person", true);
  pl::getInstance()->update(30);
  rclcpp::spin_some(pl::getInstance()->get_node_base_interface());

  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_DEBUG(node_->get_logger(), "IsPointing ticked");
    config().blackboard->get("tf_buffer", tf_buffer_);
    config().blackboard->get("tf_static_broadcaster", tf_static_broadcaster_);
  }

  getInput("person_id", person_id_);

  auto detections = pl::getInstance()->get_by_type("person");

  if (detections.empty()) {
    // RCLCPP_INFO(node_->get_logger(), "No detections");
    return BT::NodeStatus::FAILURE;
  }

  perception_system_interfaces::msg::Detection best_detection;

  std::sort(
    detections.begin(), detections.end(), [this](const auto & a, const auto & b) {
      return perception_system::diffIDs(this->person_id_, a.color_person) <
      perception_system::diffIDs(this->person_id_, b.color_person);
    });

  best_detection = detections[0];

  RCLCPP_INFO(
    node_->get_logger(), "[IsPointing] Best detection: %s, color_person: %ld, pointing: %d",
    best_detection.unique_id.c_str(), best_detection.color_person,
    best_detection.pointing_direction);

  int dev = publicTF_map2object(best_detection);
  if (dev == -1) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("bag_frame", bag_frame_);
  bag_frame_.clear();
  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsPointing>("IsPointing");
}
