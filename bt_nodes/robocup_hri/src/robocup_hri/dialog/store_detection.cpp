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

#include "robocup_hri/dialog/store_detection.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace robocup_hri
{

StoreDetection::StoreDetection(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
}

void StoreDetection::halt() {RCLCPP_INFO(node_->get_logger(), "[StoreDetection] halted");}

BT::NodeStatus StoreDetection::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "[StoreDetection} ticked");
  rclcpp::spin_some(node_->get_node_base_interface());

  getInput("current_name", current_name_);
  getInput("drink", current_drink_);
  getInput("guest_id", current_id_);
  getInput("guest_color_id", current_color_id_);
  getInput("guest_description", current_description_);

  if (current_name_.empty() || current_drink_.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  if (current_id_ == "1") {
    name_1_ = current_name_;
    drink_1_ = current_drink_;
    setOutput("name_1", name_1_);
    setOutput("drink_1", drink_1_);
    setOutput("guest_color_id_1", current_color_id_);
    setOutput("guest_description_id_1", current_description_);
    return BT::NodeStatus::SUCCESS;
  } else if (current_id_ == "2") {
    name_2_ = current_name_;
    drink_2_ = current_drink_;
    setOutput("name_2", name_2_);
    setOutput("drink_2", drink_2_);
    setOutput("guest_color_id_2", current_color_id_);
    setOutput("guest_description_id_2", current_description_);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robocup_hri

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<robocup_hri::StoreDetection>("StoreDetection");
}
