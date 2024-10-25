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

#include "configuration/init_carry.hpp"

namespace configuration
{

InitCarry::InitCarry(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  node_->declare_parameter("cam_frame", "head_front_camera_rgb_optical_frame");
  node_->declare_parameter("home_position", std::vector<double>{0.0, 0.0, 0.0});
  node_->declare_parameter("home_pose", "home");
  node_->declare_parameter("offer_pose", "offer");
  node_->declare_parameter("person_id", 001122334455);
  node_->declare_parameter("x_axis_max", 6.5);
  node_->declare_parameter("x_axis_min", -3.0);
  node_->declare_parameter("y_axis_max", 7.1);
  node_->declare_parameter("y_axis_min", -2.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

BT::NodeStatus InitCarry::tick()
{
  RCLCPP_INFO(node_->get_logger(), "InitCarry ticked");
  std::vector<double> pose_;

  if (
    node_->has_parameter("cam_frame") && node_->has_parameter("home_position") &&
    node_->has_parameter("home_pose") && node_->has_parameter("offer_pose") &&
    node_->has_parameter("person_id"))
  {
    try {
      node_->get_parameter("cam_frame", cam_frame_);
      node_->get_parameter("home_position", pose_);
      node_->get_parameter("home_pose", home_pose_);
      node_->get_parameter("offer_pose", offer_pose_);
      node_->get_parameter("person_id", person_id);
      node_->get_parameter("x_axis_max", x_axis_max_);
      node_->get_parameter("x_axis_min", x_axis_min_);
      node_->get_parameter("y_axis_max", y_axis_max_);
      node_->get_parameter("y_axis_min", y_axis_min_);

      home_position_.header.frame_id = "map";
      home_position_.pose.position.x = pose_[0];
      home_position_.pose.position.y = pose_[1];

      tf2::Quaternion q;
      q.setRPY(0, 0, pose_[2]);

      home_position_.pose.orientation = tf2::toMsg(q);

      setOutput("cam_frame", cam_frame_);
      setOutput("home_pose", home_pose_);
      setOutput("offer_pose", offer_pose_);
      setOutput("person_id", person_id);
      setOutput("home_position", home_position_);
      setOutput("x_axis_max", x_axis_max_);
      setOutput("x_axis_min", x_axis_min_);
      setOutput("y_axis_max", y_axis_max_);
      setOutput("y_axis_min", y_axis_min_);

      config().blackboard->set("tf_buffer", tf_buffer_);
      config().blackboard->set("tf_listener", tf_listener_);
      config().blackboard->set("tf_broadcaster", tf_broadcaster_);
      config().blackboard->set("tf_static_broadcaster", tf_static_broadcaster_);

      RCLCPP_INFO(node_->get_logger(), "InitCarry ticked and parameters set");

      return BT::NodeStatus::SUCCESS;
    } catch (std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "InitCarry: some parameters are missing");
      return BT::NodeStatus::FAILURE;
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "InitCarry: some parameters are missing");
    return BT::NodeStatus::FAILURE;
  }
}

void InitCarry::halt() {RCLCPP_INFO(node_->get_logger(), "InitCarry halted");}

}  // namespace configuration

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<configuration::InitCarry>("InitCarry");
}
