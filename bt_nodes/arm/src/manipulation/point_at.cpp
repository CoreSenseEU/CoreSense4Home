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

#include "arm/manipulation/point_at.hpp"
#include <math.h> 

namespace manipulation
{

using namespace std::chrono_literals;
using namespace std::placeholders;

PointAt::PointAt(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: manipulation::BtActionNode<manipulation_interfaces::action::MoveEndEffector>(xml_tag_name, action_name, conf)
{
}

void PointAt::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "PointAt ticked");

  config().blackboard->get("tf_buffer", tf_buffer_);

  getInput("pose_to_point", pose_to_point_);
  getInput("tf_frame", tf_frame_);
  getInput("base_frame", base_frame_);

  if (tf_frame_.empty() && pose_to_point_) {
    goal_.pose = *pose_to_point_;
    return;
  } else if (!tf_frame_.empty()) {    
    try {
    transform_ = tf_buffer_->lookupTransform(base_frame_, tf_frame_, tf2::TimePointZero, tf2::durationFromSec(2));
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "Could not transform %s to %s: %s", tf_frame_.c_str(), base_frame_.c_str(), ex.what());
      setStatus(BT::NodeStatus::FAILURE);
      return;
    }
    goal_.pose.pose.position.x = 0.5;
    auto y = 0.5 / std::sin(std::atan2(transform_.transform.translation.y, transform_.transform.translation.x));
    goal_.pose.pose.position.y = (std::isnan(y) || std::isinf(y)) ? 0.0 : (y); 
    goal_.pose.pose.position.z = 0.5;
    goal_.pose.pose.orientation = transform_.transform.rotation;
    goal_.pose.header.frame_id = transform_.header.frame_id;
    return;
  }

  
}

BT::NodeStatus PointAt::on_success() {return BT::NodeStatus::SUCCESS;}

}  // namespace manipulation
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<manipulation::PointAt>(name, "move_end_effector", config);
    };

  factory.registerBuilder<manipulation::PointAt>("PointAt", builder);
}
