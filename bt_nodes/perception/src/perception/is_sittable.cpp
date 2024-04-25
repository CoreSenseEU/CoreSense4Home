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

#include "perception/is_sittable.hpp"

#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "perception_system/PerceptionUtils.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

using pl = perception_system::PerceptionListener;

IsSittable::IsSittable(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  getInput("cam_frame", camera_frame_);
}



BT::NodeStatus IsSittable::tick()
{
  pl::getInstance()->set_interest("person", true);
  pl::getInstance()->set_interest("", true);
  pl::getInstance()->update(30);
  rclcpp::spin_some(pl::getInstance()->get_node_base_interface());

  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_DEBUG(node_->get_logger(), "IsSittable ticked");
    config().blackboard->get("tf_static_broadcaster", tf_static_broadcaster_);
  }

  auto person_detections = pl::getInstance()->get_by_type("person");
  auto chair_detections = pl::getInstance()->get_by_type("");

  if (person_detections.empty() && chair_detections.empty()) {
    // RCLCPP_INFO(node_->get_logger(), "No detections");
    return BT::NodeStatus::RUNNING;
  } else if (!person_detections.empty()) {
    is_person_= true;
  }

  for (auto const & object: sit_objects_) {
    is_place_to_sit_ = check_object_class(object, chair_detections);
    if (is_place_to_sit_) {
      place_to_sit_ = object;
      break;
    }    
  }

  if (!is_place_to_sit_) {
    return BT::NodeStatus::FAILURE;
  }

  if (is_person_) {
    if (check_free_space(retrieve_bb("person", person_detections), retrieve_bb(place_to_sit_, chair_detections))) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;

}

bool IsSittable::check_object_class(const std::string & obj, const std::vector<perception_system_interfaces::msg::Detection> & msg)
{
  bool ret = false;

  for (auto const & result : msg )
  {
    if (result.class_name == obj && result.score >= threshold_) {
      ret = true;
    }
  }

  return ret;
}

bool check_free_space(const geometry_msgs::msg::Point & bb1, const geometry_msgs::msg::Point & bb2 )
{
  const auto rect1 = cv::Rect(bb1.x/2, bb1.y/2, bb1.x, bb1.y);
  const auto rect2 = cv::Rect(bb2.x/2, bb2.y/2, bb2.x, bb2.y);
 
  std::vector<int> vect = {rect1.area(), rect2.area()};
  std::sort(vect.begin(), vect.end());

  if (!((rect1 & rect2).area() > vect.back()*0.375)) {
    return true;
  }

  if (((rect1 | rect2).area() > vect[0]*2)) {
    return true;
  }
  return false;
}

geometry_msgs::msg::Point 
IsSittable::retrieve_bb(const std::string & obj, const std::vector<perception_system_interfaces::msg::Detection> & msg)
{
  geometry_msgs::msg::Point bb;


  for (auto const & result : msg )
  {
    if (result.class_name == obj && result.score >= threshold_) {
      bb = result.bbox2d;
    }
  }

  return bb;
}

}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsSittable>("IsSittable");
}
