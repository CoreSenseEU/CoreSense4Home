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

#include "perception/IsDetected.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"


namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsDetected::IsDetected(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
tf_buffer_(),
tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);

  getInput("entity", entity_);
  getInput("confidence", threshold_);
  getInput("max_entities", max_entities_); 
  getInput("order", order_);
  getInput("max_depth", max_depth_);
}

BT::NodeStatus
IsDetected::tick()
{
  RCLCPP_INFO(node_->get_logger(), "IsDetected ticked");

  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_.lookupTransform("map", entity_, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "Could not transform entity to map: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;

  config().blackboard->set("person", pose);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<perception::IsDetected>("IsDetected");
}
