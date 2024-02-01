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

#include "go2object/IsDetected.hpp"

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
tf_listener_(tf_buffer_),
entity_("ND"),
threshold_(0.5),
max_entities_(1),
order_("nearest_first"),
max_depth_(3.0)
{
  config().blackboard->get("node", node_);

  getInput("entity", entity_);
  getInput("confidence", threshold_);
  getInput("max_entities", max_entities_); 
  getInput("order", order_);
  getInput("max_depth", max_depth_);

  detection3D_sub_ = node_->create_subscription<vision_msgs::msg::Detection3DArray>(
    "output_detection_3d", 100, std::bind(&IsDetected::detection_callback_, this, _1));
}

void IsDetected::detection_callback_(vision_msgs::msg::Detection3DArray::UniquePtr msg)
{
  last_detection3D_ = std::move(msg);
}

BT::NodeStatus
IsDetected::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "IsDetected ticked");
  bool is_there = false, in_range = false;
  int entities_detected = 0;
  std::vector<std::string> detected_frames;

  getInput("entity", entity_);
  getInput("confidence", threshold_);

  if (last_detection3D_ == nullptr) {
    RCLCPP_INFO(node_->get_logger(), "No 3D detection yet");
    return BT::NodeStatus::FAILURE;
  }

  auto elapsed = node_->now() - rclcpp::Time(last_detection3D_->header.stamp);

  if (elapsed > 1s) {
    RCLCPP_DEBUG(node_->get_logger(), "No 3D detection in the last second");
    return BT::NodeStatus::FAILURE;
  }

  
  if (order_ == "nearest_first") { // otherwise from left to right
    std::sort(last_detection3D_->detections.begin(), last_detection3D_->detections.end(),
      [](const vision_msgs::msg::Detection3D& a, const vision_msgs::msg::Detection3D& b) {
        return a.bbox.center.position.z < b.bbox.center.position.z;
      });
  } 

  std::string source_frame = last_detection3D_->header.frame_id;
  for (const auto& detection : last_detection3D_->detections) {
    std::string class_id = detection.results[0].hypothesis.class_id;
    double score = detection.results[0].hypothesis.score;
    
    RCLCPP_DEBUG(node_->get_logger(), "Detected object (%s). Score: %.2f", class_id.c_str(), score);
    
    in_range = is_there = detection.bbox.center.position.z <= max_depth_;

    if (!in_range) {
      RCLCPP_DEBUG(node_->get_logger(), "Object out of range (%.2f)", detection.bbox.center.position.z);
      continue;
    }
    if (entity_ != "ND") {
      is_there = (detection.results[0].hypothesis.class_id == entity_)
              && (detection.results[0].hypothesis.score >= threshold_);
    }


    if (is_there && (entities_detected < max_entities_)) { // TF entity-map to be published
      entities_detected++;
      RCLCPP_INFO(node_->get_logger(), "I see the %s (%d)!", entity_.c_str(), entities_detected);
      try{;
        tf2_ros::StaticTransformBroadcaster tf_broadcaster(node_);
        geometry_msgs::msg::TransformStamped detection_tf;
        geometry_msgs::msg::PoseStamped map_pose, camera_pose;

        camera_pose.header.frame_id = source_frame;
        camera_pose.header.stamp = node_->now();
        camera_pose.pose.position.x = detection.bbox.center.position.x;
        camera_pose.pose.position.y = detection.bbox.center.position.y;
        camera_pose.pose.position.z = detection.bbox.center.position.z;

        tf2::doTransform(camera_pose, map_pose,
                tf_buffer_.lookupTransform("map", source_frame, tf2::TimePointZero));

        detection_tf.header = map_pose.header;
        // detection_tf.child_frame_id = entity_ + "_" + std::to_string(entities_detected);
        detection_tf.child_frame_id = entity_ + "_" + detection.id; // YOLO identifier
        detection_tf.transform.translation.x = map_pose.pose.position.x;
        detection_tf.transform.translation.y = map_pose.pose.position.y;
        detection_tf.transform.translation.z = map_pose.pose.position.z;

        tf_broadcaster.sendTransform(detection_tf);
        detected_frames.push_back(detection_tf.child_frame_id);
      
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
        return BT::NodeStatus::FAILURE;
      }

      if (entities_detected == max_entities_) {
        RCLCPP_DEBUG(node_->get_logger(), "All %ss detected!", entity_.c_str());
        break;
      }
      
    }

  }

  if (!is_there) {
    RCLCPP_INFO(node_->get_logger(), "I CANNOT see any %s!", entity_.c_str());
    return BT::NodeStatus::FAILURE;
  } else {
    setOutput("frames", detected_frames);
    return BT::NodeStatus::SUCCESS;
  }

}

}  // namespace go2object


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<go2object::IsDetected>("IsDetected");
}
