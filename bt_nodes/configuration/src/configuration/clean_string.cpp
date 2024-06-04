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

#include "configuration/clean_string.hpp"

namespace configuration
{

CleanString::CleanString(
  const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
}

BT::NodeStatus CleanString::tick()
{
  getInput("string_to_clean", string_to_clean_);
  if (string_to_clean_.empty()) 
  { 
    return BT::NodeStatus::FAILURE;
  }

  string_to_clean_.erase(std::remove_if(string_to_clean_.begin(), string_to_clean_.end(), [](unsigned char c){ return !std::isalnum(c); } ), string_to_clean_.end());
  std::transform(string_to_clean_.begin(), string_to_clean_.end(), string_to_clean_.begin(),
    [](unsigned char c){ return std::tolower(c); });

  result_ = string_to_clean_;
  setOutput("result", result_);
  return BT::NodeStatus::SUCCESS;
}

void CleanString::halt() {}

}  // namespace configuration

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<configuration::CleanString>("CleanString");
}
