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

#include "configuration/increment_variable.hpp"

namespace configuration
{

IncrementVariable::IncrementVariable(
  const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
}

BT::NodeStatus IncrementVariable::tick()
{
  std::string id_{"0"};
  getInput("variable", id_);
  if (id_ == "0") {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("variable", std::to_string(std::stoi(id_) + 1));
  return BT::NodeStatus::SUCCESS;
}

void IncrementVariable::halt() {}

}  // namespace configuration

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<configuration::IncrementVariable>("IncrementVariable");
}
