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

InitCarry::InitCarry(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
}

BT::NodeStatus
InitCarry::tick()
{
  auto list = node_->list_parameters({}, 1);

  std::vector<double> pose_;
  if (node_->has_parameter("cam_frame")
     && node_->has_parameter("home_position")
     && node_->has_parameter("home_pose")
     && node_->has_parameter("offer_pose")
     && node_->has_parameter("person_id")) 
  {
    try 
    {
      node_->get_parameter("cam_frame", cam_frame_);
      node_->get_parameter("home_position", pose_);
      node_->get_parameter("home_pose", home_pose_);
      node_->get_parameter("offer_pose", offer_pose_);
      node_->get_parameter("person_id", person_id);

      home_position_.header.frame_id = "map";
      home_position_.pose.position.x = pose_[0];
      home_position_.pose.position.y = pose_[1];

      tf2::Quaternion q;
      q.setRPY(0, 0, pose_[2]);

      home_position_.pose.orientation = tf2::toMsg(q);

      config().blackboard->set("cam_frame", cam_frame_);
      config().blackboard->set("home_pose", home_pose_);
      config().blackboard->set("offer_pose", offer_pose_);
      config().blackboard->set("person_id", person_id);
      config().blackboard->set("home_position", home_position_);

      RCLCPP_INFO(node_->get_logger(), "InitCarry ticked and parameters set");      
      return BT::NodeStatus::SUCCESS;
    } 
    catch (std::exception & e) 
    {
      RCLCPP_ERROR(node_->get_logger(), "InitCarry: some parameters are missing");
      return BT::NodeStatus::FAILURE;
    }
  } 
  else 
  {
    RCLCPP_ERROR(node_->get_logger(), "InitCarry: some parameters are missing");
    return BT::NodeStatus::FAILURE;
  }
}

void
InitCarry::halt()
{
  RCLCPP_INFO(node_->get_logger(), "InitCarry halted");
}

}  // namespace configuration

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<configuration::InitCarry>("InitCarry");
}
