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

SetStartPosition::SetStartPosition(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
}

BT::NodeStatus SetStartPosition::tick()
{
  auto buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  auto static_broadcaster = config().blackboard->get<std::shared_ptr<tf2_ros::StaticTransformBroadcaster>>("tf_static_broadcaster");

  geometry_msgs::msg::TransformStamped t;

  t = buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);

  std::string frame_name;
  getInput<std::string>("frame_name", frame_name);

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = frame_name;
    transformStamped.transform.translation.x = t.transform.translation.x;
    transformStamped.transform.translation.y = t.transform.translation.y;
    transformStamped.transform.rotation = t.transform.rotation;

    static_broadcaster->sendTransform(transformStamped);

  RCLCPP_INFO(node_->get_logger(), "SetStartPosition ticked");

  return BT::NodeStatus::SUCCESS;
}

void SetStartPosition::halt() {RCLCPP_INFO(node_->get_logger(), "SetStartPosition halted");}

}  // namespace configuration

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<configuration::SetStartPosition>("SetStartPosition");
}
