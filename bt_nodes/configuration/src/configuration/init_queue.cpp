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

#include "configuration/init_queue.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <vector>

namespace configuration 
{

InitProtectedQueue::InitProtectedQueue(const std::string &xml_tag_name,
                       const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(xml_tag_name, conf) {
      config().blackboard->get("node", node_);
    }

void InitProtectedQueue::halt() {}

BT::NodeStatus InitProtectedQueue::tick()
{
    RCLCPP_INFO(node_->get_logger(), "InitProtectedQueue ticked");

    getInput<std::string>("port", port_);

    std::list<geometry_msgs::msg::TransformStamped> queue;

    std::shared_ptr<BT::ProtectedQueue<geometry_msgs::msg::TransformStamped>> protected_queue = std::make_shared<BT::ProtectedQueue<geometry_msgs::msg::TransformStamped>>();
    protected_queue->items = queue;

    config().blackboard->set(port_, protected_queue);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace configuration

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<configuration::InitProtectedQueue>("InitProtectedQueue");
}
