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

#include "configuration/init_groceries.hpp"

namespace configuration
{

InitGroceries::InitGroceries(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  node_->declare_parameter("cam_frame", "head_front_camera_rgb_optical_frame");
  node_->declare_parameter("manipulation_frame", "base_link");
  // node_->declare_parameter("party_wp",  std::vector<double>{0.0, 0.0, 0.0});
  // node_->declare_parameter("entrance_wp",  std::vector<double>{0.0, 0.0, 0.0});
  node_->declare_parameter("host_name", "John Doe");
  node_->declare_parameter("host_drink", "beer");
  // node_->declare_parameter("waypoints_names", std::vector<std::string>{});

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

BT::NodeStatus InitGroceries::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "[InitGroceries] ticked");

  if (
    node_->has_parameter("cam_frame") && node_->has_parameter("manipulation_frame") &&
    node_->has_parameter("host_name") && node_->has_parameter("host_drink"))
  // && node_->has_parameter("waypoints_names"))
  // node_->has_parameter("party_wp") && node_->has_parameter("entrance_wp"))
  {
    node_->get_parameter("cam_frame", cam_frame_);
    node_->get_parameter("manipulation_frame", manipulation_frame_);
    node_->get_parameter("host_name", host_name_);
    node_->get_parameter("host_drink", host_drink_);
    // node_->get_parameter("waypoints_names", wp_names_);

    // for (auto wp : wp_names_) {
    //   node_->declare_parameter("waypoints." + wp, std::vector<double>());
    //   std::vector<double> wp_pos;
    //   node_->get_parameter("waypoints." + wp, wp_pos);
    //   geometry_msgs::msg::TransformStamped transform_msg;
    //   tf2::Quaternion q;

    //   if (wp.find("entrance")!=std::string::npos) {
    //     setOutput("entrance_wp", wp);
    //   } else if (wp.find("party")!=std::string::npos) {
    //     setOutput("party_wp", wp);
    //   }

    //   q.setRPY(0, 0, wp_pos[2]);
    //   transform_msg.header.frame_id = "map";

    //   transform_msg.child_frame_id = wp;
    //   transform_msg.transform.translation.x = wp_pos[0];
    //   transform_msg.transform.translation.y = wp_pos[1];
    //   transform_msg.transform.rotation = tf2::toMsg(q);
    //   tf_static_broadcaster_->sendTransform(transform_msg);
    //   rclcpp::spin_some(node_);
    // }

    geometry_msgs::msg::TransformStamped transform_msg;

    transform_msg.header.frame_id = "base_link";

    transform_msg.child_frame_id = "attention_home";
    transform_msg.transform.translation.x = 1.5;
    transform_msg.transform.translation.z = 1.5;
    tf_static_broadcaster_->sendTransform(transform_msg);
    rclcpp::spin_some(node_->get_node_base_interface());

    std::string current_guest = "1";
    setOutput("cam_frame", cam_frame_);
    setOutput("manipulation_frame", manipulation_frame_);
    setOutput("host_name", host_name_);
    RCLCPP_INFO(node_->get_logger(), "Host name: %s", host_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Host drink %s", host_drink_.c_str());
    setOutput("host_drink", host_drink_);
    setOutput("current_guest", current_guest);
    setOutput("tf_buffer", tf_buffer_);
    setOutput("tf_listener", tf_listener_);
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

void InitGroceries::halt() {RCLCPP_INFO(node_->get_logger(), "InitGroceries halted");}

}  // namespace configuration

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<configuration::InitGroceries>("InitGroceries");
}
