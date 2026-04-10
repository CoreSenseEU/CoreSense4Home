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

#include "perception/ExtractHandoverAlignment.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace perception
{

ExtractHandoverAlignment::ExtractHandoverAlignment(
  const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  detected_objs_sub_ = node_->create_subscription<yolo_msgs::msg::DetectionArray>(
    "detections_3d", 100, std::bind(&ExtractHandoverAlignment::detection_callback_, this, _1));
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, std::bind(&ExtractHandoverAlignment::joint_state_callback_, this, _1));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ExtractHandoverAlignment::halt()
{
  RCLCPP_INFO(node_->get_logger(), "ExtractHandoverAlignment halted");
}

void ExtractHandoverAlignment::detection_callback_(yolo_msgs::msg::DetectionArray::UniquePtr msg)
{
  last_detected_objs_ = std::move(msg);
}

void ExtractHandoverAlignment::joint_state_callback_(sensor_msgs::msg::JointState::UniquePtr msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "torso_lift_joint") {
      current_torso_height_ = msg->position[i];
      torso_height_received_ = true;
      break;
    }
  }
}

BT::NodeStatus ExtractHandoverAlignment::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[ExtractHandoverAlignment] ticked");
  getInput("interest_class", interest_class_);
  double target_z_distance = 0.40; // Default 40cm in Z from arm_4_link
  double target_y_distance = -0.30; // Default -30cm in Y from arm_4_link
  getInput("target_z_distance", target_z_distance);
  getInput("target_y_distance", target_y_distance);

  // rclcpp::spin_some(node_->get_node_base_interface());

  if (!torso_height_received_) {
    RCLCPP_ERROR(node_->get_logger(), "[ExtractHandoverAlignment] No torso joint state received");
    return BT::NodeStatus::FAILURE;
  }

  if (last_detected_objs_ == nullptr) {
    RCLCPP_ERROR(node_->get_logger(), "[ExtractHandoverAlignment] No objects detection yet");
    return BT::NodeStatus::FAILURE;
  }

  auto elapsed = node_->now() - rclcpp::Time(last_detected_objs_->header.stamp);
  if (elapsed > 2s) {
    RCLCPP_ERROR(node_->get_logger(), "[ExtractHandoverAlignment] No recent detections");
    return BT::NodeStatus::FAILURE;
  }

  for (auto const & detected_object : last_detected_objs_->detections) {
    if (detected_object.class_name != interest_class_ && interest_class_ != "") {
      RCLCPP_DEBUG(node_->get_logger(), "Ignoring object %s", detected_object.class_name.c_str());
      continue;
    }

    tf2::Transform camera_2_object;
    camera_2_object.setOrigin(
      tf2::Vector3(
        detected_object.bbox3d.center.position.x, detected_object.bbox3d.center.position.y,
        detected_object.bbox3d.center.position.z));
    camera_2_object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    // Get transform from arm_4_link to object frame
    geometry_msgs::msg::TransformStamped arm_2_camera_msg;
    try {
      arm_2_camera_msg = tf_buffer_->lookupTransform(
        "arm_4_link", detected_object.bbox3d.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        node_->get_logger(), "TF Error looking up %s: %s",
        detected_object.bbox3d.frame_id.c_str(), ex.what());
      return BT::NodeStatus::FAILURE;
    }

    tf2::Transform arm_2_camera;
    tf2::fromMsg(arm_2_camera_msg.transform, arm_2_camera);
    tf2::Transform arm_2_object = arm_2_camera * camera_2_object;

    // The pose in arm_4_link
    double obj_x = arm_2_object.getOrigin().x(); // Left/Right from arm
    double obj_y = arm_2_object.getOrigin().y(); // Up/Down relative to arm base
    double obj_z = arm_2_object.getOrigin().z(); // Forward/Back relative to arm

    // Calculate required movements
    // Z is forward distance from arm_4_link. If target is 40cm, and object is at 20cm, we need to move the base backward (negative X base movement) by 20cm.
    // Base movement required = object Z distance - target Z distance
    double base_x_movement = obj_z - target_z_distance;

    // Y is up/down from arm_4_link. If target is -30cm (object below arm 4 link), and object is at -50cm, torso needs to lift by 20cm (positive Z).
    // Note: ROS standard is Z is up, but here Y might be up/down depending on arm_4_link orientation. Assuming standard where object Z is forward, Y is down/up.
    // Let's print out the values to be safe, but typically in optical frame Z is forward, Y is down, X is right. From arm_4_link it might be different.
    // Assuming arm_4_link Z is along the arm (forward), X is down/up. We will output the raw differences and the robot control node can handle the mapping.

    // Torso Z movement required = object Y distance - target Y distance
    // If target is -30 (object below), and object is at -50 (even further below), we need to lower the torso? No, if it's -50 and we want it at -30, we must move torso DOWN by 20 so relative Y becomes -30. Wait.
    // If torso goes DOWN, arm goes DOWN. Object Y relative to arm becomes less negative.
    // If torso goes UP, arm goes UP. Object Y relative to arm becomes more negative.
    // So torso_z_movement = target_y_distance - obj_y  (or vice versa depending on axes).
    // For now, let's output the error directly:
    double torso_z_error = obj_y - target_y_distance;

    double new_torso_height = current_torso_height_ + torso_z_error;

    // Limits
    double min_height = 0.11;
    double max_height = 0.35;
    if (new_torso_height < min_height) {
      new_torso_height = min_height;
      RCLCPP_WARN(node_->get_logger(), "[HandoverAlign] Torso height limited to min (0.0)");
    } else if (new_torso_height > max_height) {
      new_torso_height = max_height;
      RCLCPP_WARN(node_->get_logger(), "[HandoverAlign] Torso height limited to max (0.35)");
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[HandoverAlign] Object %s found in arm_4_link. Z: %.2f (target %.2f), Y: %.2f (target %.2f)",
      detected_object.class_name.c_str(), obj_z, target_z_distance, obj_y, target_y_distance);
    RCLCPP_INFO(
      node_->get_logger(),
      "[HandoverAlign] Calculated Base X diff: %.2f, Torso Z diff: %.2f, Absolute Torso Height: %.2f",
      base_x_movement, torso_z_error, new_torso_height);

    ExtractHandoverAlignment::setOutput("base_x_movement", base_x_movement);
    ExtractHandoverAlignment::setOutput("torso_z_movement", new_torso_height);

    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "[ExtractHandoverAlignment] Object of interest %s not found in scene", interest_class_.c_str());
  return BT::NodeStatus::FAILURE;
}

}  // namespace perception

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<perception::ExtractHandoverAlignment>("ExtractHandoverAlignment");
}
