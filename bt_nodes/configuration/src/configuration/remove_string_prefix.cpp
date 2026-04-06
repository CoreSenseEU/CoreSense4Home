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

#include "configuration/remove_string_prefix.hpp"

namespace configuration
{

RemoveStringPrefix::RemoveStringPrefix(
  const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
}

BT::NodeStatus RemoveStringPrefix::tick()
{
  getInput("string_to_remove", string_to_remove_);
  getInput("prefix", prefix_);
  if (string_to_remove_.empty() || prefix_.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  size_t pos = string_to_remove_.find(prefix_);
  if (pos != std::string::npos) {
    result_ = string_to_remove_.substr(pos + prefix_.length());
  } else {
    result_ = string_to_remove_;
  }

  setOutput("result", result_);
  return BT::NodeStatus::SUCCESS;
}

void RemoveStringPrefix::halt() {}

}  // namespace configuration

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<configuration::RemoveStringPrefix>("RemoveStringPrefix");
}
