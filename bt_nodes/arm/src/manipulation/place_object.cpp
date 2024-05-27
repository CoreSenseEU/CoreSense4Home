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

#include "arm/manipulation/place_object.hpp"

#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "moveit_msgs/msg/collision_object.hpp"

namespace manipulation
{

using namespace std::chrono_literals;
using namespace std::placeholders;

PlaceObject::PlaceObject(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: manipulation::BtActionNode<manipulation_interfaces::action::Place,
  rclcpp_cascade_lifecycle::CascadeLifecycleNode>(xml_tag_name, action_name, conf)
{
}

void PlaceObject::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "PlaceObject ticked");
  // tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  getInput("object_to_place", object_);
  // getInput("tf_to_place", tf_to_place_);
  getInput("place_pose", place_pose_);
  getInput("base_frame", base_frame_);


  // try
  // {
  //   transform_to_place_ = tf_buffer_->lookupTransform(base_frame_, tf_to_place_, tf2::TimePointZero, tf2::durationFromSec(1.0));
  // }
  // catch(const std::exception& e)
  // {
  //   setStatus(BT::NodeStatus::FAILURE);
  //   return;
  // }
  // place_pose_.header.frame_id = base_frame_;
  // place_pose_.pose.position.x = transform_to_place_.transform.translation.x;
  // place_pose_.pose.position.y = transform_to_place_.transform.translation.y;
  // place_pose_.pose.position.z = transform_to_place_.transform.translation.z;
  // place_pose_.pose.orientation = transform_to_place_.transform.rotation;  

  goal_.attached_object = *object_;
  goal_.place_pose = place_pose_;

}

BT::NodeStatus PlaceObject::on_success() 
{
  if (result_.result->success) {
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Place failed");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace manipulation
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<manipulation::PlaceObject>(name, "/place", config);
    };

  factory.registerBuilder<manipulation::PlaceObject>("PlaceObject", builder);
}
