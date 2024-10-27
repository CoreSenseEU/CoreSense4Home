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

#include "cs4home_core/Efferent.hpp"

namespace cs4home_core
{

Efferent::Efferent(rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
: parent_(parent)
{
}

bool
Efferent::create_publisher(const std::string & topic, const std::string & type)
{
  auto pub = rclcpp::create_generic_publisher(
    parent_->get_node_topics_interface(), topic, type, 100);

  pubs_.push_back(pub);

  return true;
}

}  // namespace cs4home_core
