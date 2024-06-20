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

#include "configuration/SetStartPosition.hpp"

namespace configuration
{

SetStartPosition::SetStartPosition(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
}

BT::NodeStatus SetStartPosition::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "SetStartPosition ticked");

  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_DEBUG(node_->get_logger(), "SetStartPosition idle");
    buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    static_broadcaster_ =
      config().blackboard->get<std::shared_ptr<tf2_ros::StaticTransformBroadcaster>>(
      "tf_static_broadcaster");
    getInput("x_offset", x_offset_);
    getInput("y_offset", y_offset_);
  }


  geometry_msgs::msg::TransformStamped transformStamped;

  try {
    transformStamped = buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "start";
  transformStamped.transform.translation.x = transformStamped.transform.translation.x + x_offset_;
  transformStamped.transform.translation.y = transformStamped.transform.translation.y + y_offset_;
  transformStamped.transform.rotation = transformStamped.transform.rotation;

  static_broadcaster_->sendTransform(transformStamped);

  setOutput("initial_pose", "start");
  rclcpp::spin_some(node_->get_node_base_interface());


  return BT::NodeStatus::SUCCESS;
}

void SetStartPosition::halt() {RCLCPP_INFO(node_->get_logger(), "SetStartPosition halted");}

}  // namespace configuration

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<configuration::SetStartPosition>("SetStartPosition");
}
