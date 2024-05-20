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

#include "perception/extract_collision_scene.hpp"

#include <string>
#include <utility>

#include "perception_system_interfaces/srv/isolate_pc_background.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

ExtractCollisionScene::ExtractCollisionScene(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: perception::BtServiceNode<perception_system_interfaces::srv::IsolatePCBackground,
  rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
}

void ExtractCollisionScene::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "ExtractCollisionScene ticked");

  moveit_msgs::msg::CollisionObject::SharedPtr selected_object;
  getInput("selected_object", selected_object);

  request_->classes = {selected_object->id.substr(0, selected_object->id.find('_'))};
}

void ExtractCollisionScene::on_result()
{
  if (result_.success) {
    std::cout << "Success" << std::endl;
    setStatus(BT::NodeStatus::SUCCESS);
  } else {
    std::cout << "Failure" << std::endl;
    // setOutput("listen_text", result_.result->text);
    setStatus(BT::NodeStatus::FAILURE);
  }
}

}  // namespace perception

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<perception::ExtractCollisionScene>(
        name, "perception_system/isolate_pc_background", config);
    };

  factory.registerBuilder<perception::ExtractCollisionScene>("ExtractCollisionScene", builder);
}
