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

#include "configuration/init_restaurant.hpp"

namespace configuration
{

InitRestaurant::InitRestaurant(
  const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  node_->declare_parameter("cam_frame", "head_front_camera_link_color_optical_frame");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

BT::NodeStatus InitRestaurant::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "[InitRestaurant] ticked");

  if (node_->has_parameter("cam_frame")) {
    node_->get_parameter("cam_frame", cam_frame_);

    geometry_msgs::msg::TransformStamped transform_msg;

    transform_msg.header.frame_id = "base_link";

    transform_msg.child_frame_id = "attention_home";
    transform_msg.transform.translation.x = 1.5;
    transform_msg.transform.translation.z = 1.5;
    tf_static_broadcaster_->sendTransform(transform_msg);
    rclcpp::spin_some(node_->get_node_base_interface());

    setOutput("cam_frame", cam_frame_);
    setOutput("order_1", "");
    setOutput("order_2", "");
    setOutput("order_3", "");
    setOutput("order_count", "0");
    setOutput("tf_buffer", tf_buffer_);
    setOutput("tf_broadcaster", tf_broadcaster_);
    setOutput("tf_static_broadcaster", tf_static_broadcaster_);
    setOutput("attention_home", "attention_home");

    config().blackboard->set("tf_buffer", tf_buffer_);
    config().blackboard->set("tf_listener", tf_listener_);
    config().blackboard->set("tf_broadcaster", tf_broadcaster_);
    config().blackboard->set("tf_static_broadcaster", tf_static_broadcaster_);

    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void InitRestaurant::halt() {RCLCPP_INFO(node_->get_logger(), "InitRestaurant halted");}

}  // namespace configuration

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<configuration::InitRestaurant>("InitRestaurant");
}
