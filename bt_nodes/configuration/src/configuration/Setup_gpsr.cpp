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

#include "configuration/Setup_gpsr.hpp"

namespace configuration {

SetupGPSR::SetupGPSR(const std::string &xml_tag_name,
                     const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(xml_tag_name, conf) {

  config().blackboard->get("node", node_);
    }

void SetupGPSR::halt() {}

BT::NodeStatus SetupGPSR::tick() {

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  auto tf_broadcast = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  config().blackboard->set("tf_buffer", tf_buffer);
  config().blackboard->set("tf_listener", tf_listener);
  config().blackboard->set("tf_broadcaster", tf_broadcast);


  std::vector<std::string> plugins;
  plugins.push_back("is_detected_bt_node");
  plugins.push_back("extract_object_from_scene_bt_node");
  plugins.push_back("convert_color_bt_node");
  plugins.push_back("is_gesturing_bt_node");
  plugins.push_back("filter_entity_bt_node");
  plugins.push_back("is_entity_moving_bt_node");
  plugins.push_back("extract_entity_color_bt_node");
  plugins.push_back("extract_collision_scene_bt_node");
  plugins.push_back("generate_text_from_objects_bt_node");
  plugins.push_back("speak_bt_node");
  plugins.push_back("dialogConfirmation_bt_node");
  plugins.push_back("listen_bt_node");
  plugins.push_back("query_bt_node");
  plugins.push_back("command_planning_bt_node");
  plugins.push_back("move_to_predefined_bt_node");
  plugins.push_back("pick_bt_node");
  plugins.push_back("move_to_bt_node");
  plugins.push_back("look_at_bt_node");
  plugins.push_back("configure_navigate_back_bt_node");
  plugins.push_back("pan_bt_node");
  plugins.push_back("follow_entity_bt_node");
  plugins.push_back("goal_publisher_bt_node");
  plugins.push_back("set_perception_model_bt_node");
  plugins.push_back("rotate_bt_node");
  plugins.push_back("set_head_joint_position_bt_node");
  plugins.push_back("set_torso_height_bt_node");
  plugins.push_back("filter_object_bt_node");
  plugins.push_back("ConsumeQueue");
  plugins.push_back("publish_tf_bt_node");
  plugins.push_back("init_protected_queue_bt_node");
  plugins.push_back("filter_prev_detections_bt_node");
  plugins.push_back("extract_person_description_bt_node");
  plugins.push_back("sleep_bt_node");
  plugins.push_back("count_people_bt_node");
  plugins.push_back("set_blackboard_int_bt_node");

  setOutput("plugins", plugins);

  return BT::NodeStatus::SUCCESS;
}

} // namespace configuration

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<configuration::SetupGPSR>("SetupGPSR");
}
