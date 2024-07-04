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

#ifndef CONFIGURATION__CLEAN_STRING_HPP_
#define CONFIGURATION__CLEAN_STRING_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace configuration
{

class CleanString : public BT::ActionNodeBase
{
public:
  explicit CleanString(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("string_to_clean"),
        BT::OutputPort<std::string>("result")
      });
  }

private:
  std::string string_to_clean_, result_;
};

}  // namespace configuration

#endif  // CONFIGURATION__CLEAN_STRING_HPP_
