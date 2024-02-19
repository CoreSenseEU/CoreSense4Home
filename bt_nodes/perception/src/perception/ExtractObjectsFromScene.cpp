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

#include "perception/ExtractObjectsFromScene.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals; 

namespace perception
{

ExtractObjectsFromScene::ExtractObjectsFromScene(const std::string & xml_tag_name,
                                                 const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  detected_objs_sub_ = node_->create_subscription<yolov8_msgs::msg::DetectionArray>(
    "/perception_system/objects_detection", 100, std::bind(&ExtractObjectsFromScene::detection_callback_, this, _1));
  
}

void
ExtractObjectsFromScene::halt()
{
  RCLCPP_INFO(node_->get_logger(), "ExtractObjectsFromScene halted");
}

void ExtractObjectsFromScene::detection_callback_(yolov8_msgs::msg::DetectionArray::UniquePtr msg)
{
  last_detected_objs_ = std::move(msg);
}

BT::NodeStatus
ExtractObjectsFromScene::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "ExtractObjectsFromScene ticked");
  rclcpp::spin_some(node_);

  
  if (last_detected_objs_ == nullptr) {
    RCLCPP_INFO(node_->get_logger(), "No objects detection yet");
    return BT::NodeStatus::FAILURE;
  }

  auto elapsed = node_->now() - rclcpp::Time(last_detected_objs_->header.stamp);
  double seconds = elapsed.nanoseconds()*1000;

  RCLCPP_INFO(node_->get_logger(), "Elapsed time: %f", seconds);

  if (elapsed > 1s) {
    RCLCPP_INFO(node_->get_logger(), "No objects detection in the last second");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "Objects detected");
  RCLCPP_INFO(node_->get_logger(), "Number of detected objects: %ld",last_detected_objs_->detections.size());

  std::vector<moveit_msgs::msg::CollisionObject::SharedPtr> detected_objects = {};


  auto header = last_detected_objs_->header;
  for (auto const &detected_object : last_detected_objs_->detections) {

    shape_msgs::msg::SolidPrimitive obj_solid_primitive;
    obj_solid_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    obj_solid_primitive.dimensions = {detected_object.bbox3d.size.x, 
                                      detected_object.bbox3d.size.y, 
                                      detected_object.bbox3d.size.z};

    auto obj_ptr = std::make_shared<moveit_msgs::msg::CollisionObject>();

    obj_ptr->header = header;

    obj_ptr->id = detected_object.class_name +"_" + detected_object.id;
    obj_ptr->primitives = {obj_solid_primitive};
    obj_ptr->primitive_poses = {detected_object.bbox3d.center};
    obj_ptr->operation = moveit_msgs::msg::CollisionObject::ADD;
    detected_objects.push_back(obj_ptr); 

    RCLCPP_INFO(node_->get_logger(), "Object Found Id: %s",(obj_ptr->id).c_str());
  }
  ExtractObjectsFromScene::setOutput("detected_objects",detected_objects);
  ExtractObjectsFromScene::setOutput("objects_count",detected_objects.size());
  RCLCPP_INFO(node_->get_logger(), "---------------------------------------");
  if(detected_objects.empty())
  {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<perception::ExtractObjectsFromScene>("ExtractObjectsFromScene");
}
