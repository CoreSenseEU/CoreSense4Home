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

#include "perception/FollowPerson.hpp"
#include "perception_system/PerceptionUtils.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"


namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

FollowPerson::FollowPerson(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  // config().blackboard->get("perception_listener", perception_listener_);

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

  perception_listener_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  perception_listener_->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  getInput("person_id", person_id_);
  getInput("camera_link", camera_link_);
  // person_id_ = 103020103212;
  // camera_link_ = "head_front_camera_link";
}

void
FollowPerson::halt()
{
  RCLCPP_INFO(node_->get_logger(), "FollowPerson halted");
}

int 
FollowPerson::publicTF_map2obj(const perception_system_interfaces::msg::Detection & detected_object)
{
  geometry_msgs::msg::TransformStamped map2camera_msg;
    try {
      map2camera_msg = tf_buffer_->lookupTransform(
        "map", camera_link_,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        node_->get_logger(), "Could not transform %s to %s: %s",
        "map", camera_link_.c_str(), ex.what());
        try {
          map2camera_msg = tf_buffer_->lookupTransform(
            "odom", camera_link_,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            node_->get_logger(), "Could not transform %s to %s: %s",
            "odom", camera_link_.c_str(), ex.what());
          return -1;
        }
      return -1;
    }

    tf2::Transform camera2obstacle;
    camera2obstacle.setOrigin(
      tf2::Vector3(
        detected_object.center3d.position.x,
        detected_object.center3d.position.y,
        detected_object.center3d.position.z));
    camera2obstacle.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    tf2::Transform map2camera;
    tf2::fromMsg(map2camera_msg.transform, map2camera);
 
    tf2::Transform map2obstacle = map2camera * camera2obstacle;
    // create a transform message from tf2::Transform
    geometry_msgs::msg::TransformStamped map2obstacle_msg;
    map2obstacle_msg.header.stamp = detected_object.header.stamp;
    map2obstacle_msg.header.frame_id = "map";
    map2obstacle_msg.child_frame_id = "person";
    map2obstacle_msg.transform = tf2::toMsg(map2obstacle);

    tf_broadcaster_->sendTransform(map2obstacle_msg);
    return 0;
}

BT::NodeStatus
FollowPerson::tick()
{
  perception_listener_->set_interest("person", true);
  rclcpp::spin_some(perception_listener_->get_node_base_interface());
  
  // RCLCPP_INFO(node_->get_logger(), "FollowPerson ticked");
  // // std::cout << "perception_listener_: " << perception_listener_ << std::endl;

  std::vector<perception_system_interfaces::msg::Detection> detections;
  detections = perception_listener_->get_by_type("person");
  if (!perception_listener_) {
    RCLCPP_ERROR(node_->get_logger(), "PerceptionListener not found");
  }

  if (detections.empty()) {
    RCLCPP_INFO(node_->get_logger(), "No detections");
    return BT::NodeStatus::FAILURE;
  }

  perception_system_interfaces::msg::Detection best_detection;
  // std::cout << "detections type person: " << detections.size() << std::endl;
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


  int dev = publicTF_map2obj(best_detection);
  if (dev != 0) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace perception


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<perception::FollowPerson>("FollowPerson");
}
