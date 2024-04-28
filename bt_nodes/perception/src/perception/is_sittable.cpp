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

  // delete the following: 
  pl::getInstance()->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  pl::getInstance()->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}



BT::NodeStatus IsSittable::tick()
{
  pl::getInstance()->set_interest("", true);
  pl::getInstance()->set_interest("person", true);
  pl::getInstance()->update(30);
  rclcpp::spin_some(pl::getInstance()->get_node_base_interface());

  if (status() == BT::NodeStatus::IDLE) {
    RCLCPP_DEBUG(node_->get_logger(), "[IsSittable] ticked");
    config().blackboard->get("tf_static_broadcaster", tf_static_broadcaster_);
    config().blackboard->get("tf_buffer", tf_buffer_);
    getInput("cam_frame", camera_frame_); 
    pl::getInstance()->set_interest("", true);
    pl::getInstance()->set_interest("person", true);
    pl::getInstance()->update(30);
    rclcpp::spin_some(pl::getInstance()->get_node_base_interface());
    return BT::NodeStatus::RUNNING;
  }

  auto person_detections = pl::getInstance()->get_by_type("person");
  auto chair_detections = pl::getInstance()->get_by_type("");

  if (person_detections.empty() && chair_detections.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[IsSittable] no detections found");
    return BT::NodeStatus::FAILURE;
  } else if (!person_detections.empty()) {
    is_person_= true;
  }
  RCLCPP_DEBUG(node_->get_logger(), "[IsSittable] one person detected");


  for (auto const & object: sit_objects_) {
    is_place_to_sit_ = check_object_class(object, chair_detections);
    if (is_place_to_sit_) {
      place_to_sit_ = object;
      break;
    }    
  }


  if (!is_place_to_sit_) {
    RCLCPP_ERROR(node_->get_logger(), "[IsSittable] no place to sit :(");
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_DEBUG(node_->get_logger(), "[IsSittable] one chair detected");

  if (is_place_to_sit_ && !is_person_) {
    RCLCPP_DEBUG(node_->get_logger(), "[IsSittable] no person detected but chair found");
    geometry_msgs::msg::TransformStamped map2cam_msg;
      try {
        map2cam_msg = tf_buffer_->lookupTransform("map",  camera_frame_, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(
          node_->get_logger(), "Could not transform %s to %s: %s", camera_frame_.c_str(), "base_link", ex.what());
        return BT::NodeStatus::FAILURE;
      }
      
      tf2::Transform camera2object;
      camera2object.setOrigin(
        tf2::Vector3(
          chair_detection_.center3d.position.x, chair_detection_.center3d.position.y,
          chair_detection_.center3d.position.z));

      camera2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

      tf2::Transform map2camera;
      tf2::fromMsg(map2cam_msg.transform, map2camera);

      tf2::Transform map2object = map2camera * camera2object;

      geometry_msgs::msg::TransformStamped map2object_msg;

      map2object_msg.header.frame_id = "map";
      map2object_msg.child_frame_id = place_to_sit_ + "_frame";
      map2object_msg.transform = tf2::toMsg(map2object);
      
      tf_static_broadcaster_->sendTransform(map2object_msg);
      setOutput("chair_frame", place_to_sit_ + "_frame");
    return BT::NodeStatus::SUCCESS;
  }
  
  if (is_person_) {
    person_detection_ = retrieve_detection("person", person_detections);
    chair_detection_ = retrieve_detection(place_to_sit_, chair_detections);

    if (check_free_space(person_detection_, chair_detection_)) {
      geometry_msgs::msg::TransformStamped map2cam_msg;
      try {
        map2cam_msg = tf_buffer_->lookupTransform("map",  camera_frame_, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(
          node_->get_logger(), "Could not transform %s to %s: %s", camera_frame_.c_str(), "base_link", ex.what());
        return BT::NodeStatus::FAILURE;
      }


      
      tf2::Transform camera2object;
      camera2object.setOrigin(
        tf2::Vector3(
          chair_detection_.center3d.position.x, chair_detection_.center3d.position.y,
          chair_detection_.center3d.position.z));

      camera2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

      tf2::Transform map2camera;
      tf2::fromMsg(map2cam_msg.transform, map2camera);

      tf2::Transform map2object = map2camera * camera2object;

      geometry_msgs::msg::TransformStamped map2object_msg;

      map2object_msg.header.frame_id = "map";
      map2object_msg.child_frame_id = place_to_sit_ + "_frame";
      map2object_msg.transform = tf2::toMsg(map2object);
      
      tf_static_broadcaster_->sendTransform(map2object_msg);
      setOutput("chair_frame", place_to_sit_ + "_frame");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::FAILURE;

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

bool IsSittable::check_free_space(const perception_system_interfaces::msg::Detection & person_detection,
                                  const perception_system_interfaces::msg::Detection & chair_detection)
{
  RCLCPP_DEBUG(node_->get_logger(), "[IsSittable] check_free_space");
  RCLCPP_DEBUG(node_->get_logger(), "[IsSittable] person bbox: %f %f", person_detection.bbox3d.x, person_detection.bbox3d.y);
  RCLCPP_DEBUG(node_->get_logger(), "[IsSittable] chair bbox: %f %f", chair_detection.bbox3d.x, chair_detection.bbox3d.y);
  // position and bbx are in meters, convert to cm:
  
  const auto rect1 = cv::Rect(person_detection.center3d.position.x*100 - person_detection.bbox3d.x*100/2, person_detection.center3d.position.y*100 - person_detection.bbox3d.y*100/2 , person_detection.bbox3d.x*100, person_detection.bbox3d.y*100);
  const auto rect2 = cv::Rect(chair_detection.center3d.position.x*100 - chair_detection.bbox3d.x*100/2, chair_detection.center3d.position.y*100 - chair_detection.bbox3d.y*100/2 , chair_detection.bbox3d.x*100, chair_detection.bbox3d.y*100);

  RCLCPP_DEBUG(node_->get_logger(), "[IsSittable] chair area: %d", rect2.area());
  RCLCPP_DEBUG(node_->get_logger(), "[IsSittable] person area: %d", rect1.area());
  if (rect1.area() == 0 || rect2.area() == 0) {
    RCLCPP_ERROR(node_->get_logger(), "[IsSittable] area is 0");
    return false;
  }
  std::vector<int> vect = {rect1.area(), rect2.area()};
  std::sort(vect.begin(), vect.end());

  if (!((rect1 & rect2).area() > vect.back()*0.375)) {
    RCLCPP_DEBUG(node_->get_logger(), "[IsSittable] free space");
    return true;
  }

  if (((rect1 | rect2).area() > vect[0]*2)) {
    RCLCPP_DEBUG(node_->get_logger(), "[IsSittable] free space");
    return true;
  }
  RCLCPP_DEBUG(node_->get_logger(), "[IsSittable] no free space");
  return false;
}

geometry_msgs::msg::Vector3 
IsSittable::retrieve_bb(const std::string & obj, const std::vector<perception_system_interfaces::msg::Detection> & msg)
{
  geometry_msgs::msg::Vector3 bb;


  for (auto const & result : msg )
  {
    if (result.class_name == obj && result.score >= threshold_) {
      bb = result.bbox3d;
    }
  }

  return bb;
}
geometry_msgs::msg::Pose 
IsSittable::retrieve_3d_pose(const std::string & obj, const std::vector<perception_system_interfaces::msg::Detection> & msg)
{
  geometry_msgs::msg::Pose pos;


  for (auto const & result : msg )
  {
    if (result.class_name == obj && result.score >= threshold_) {
      pos = result.center3d;
    }
  }

  return pos;
}

perception_system_interfaces::msg::Detection
IsSittable::retrieve_detection(const std::string & obj, const std::vector<perception_system_interfaces::msg::Detection> & msg)
{
  perception_system_interfaces::msg::Detection detection;

  for (auto const & result : msg )
  {
    if (result.class_name == obj && result.score >= threshold_) {
      detection = result;
    }
  }

  return detection;
  
}



}  // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::IsSittable>("IsSittable");
}
