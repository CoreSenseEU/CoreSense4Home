// Copyright 2024 Intelligent Robotics Lab
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

#include "cs4home_core/Afferent.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/serialization.hpp"

namespace cs4home_core
{

Afferent::Afferent(rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
: parent_(parent)
{
}

bool
Afferent::configure()
{
  /*std::vector<std::string> afferents;
  parent_->declare_parameter("afferent", afferents);
  parent_->get_parameter("afferent", afferents);

  for (const auto & afferent: afferents) {
    std::string topic, type;
    parent_->declare_parameter("afferent." + afferent + ".topic", topic);
    parent_->get_parameter("afferent." + afferent + ".type", type);

    if (topic == "" || type == "") {
      return false;
    }

    if (!create_subscriber(topic, type)) {
      RCLCPP_ERROR(
        parent_->get_logger(), "Error creating afferent [%s, %s]", topic.c_str(), type.c_str());
      return false;
    }
  }
*/
  return true;
}

bool
Afferent::create_subscriber(const std::string & topic, const std::string & type)
{
  auto sub = parent_->create_generic_subscription(topic, type, 100,
      [&](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        RCLCPP_INFO(parent_->get_logger(), "Llega el mensaje");
    });

  subs_.push_back(sub);

  return true;
}

}  // namespace cs4home_core
