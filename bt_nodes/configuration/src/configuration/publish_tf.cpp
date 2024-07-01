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

#include "configuration/publish_tf.hpp"


namespace configuration
{

PublishTF::PublishTF(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
    config().blackboard->get("node", node_);
    config().blackboard->get("tf_broadcaster", tf_broadcaster_);
}

void
PublishTF::halt()
{
}

BT::NodeStatus
PublishTF::tick()
{
    RCLCPP_INFO(node_->get_logger(), "PublishTF ticked");
    getInput<geometry_msgs::msg::TransformStamped>("transform", transform_);

    transform_.header.stamp = node_->now();
    tf_broadcaster_->sendTransform(transform_);
  
    setOutput("frame_id", transform_.child_frame_id);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace deferred_bt

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<configuration::PublishTF>("PublishTF");
}
