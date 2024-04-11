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

#ifndef CS4HOME_CORE__COUPLING_HPP_
#define CS4HOME_CORE__COUPLING_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace cs4home_core
{

class Coupling
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Coupling)

  explicit Coupling(rclcpp_lifecycle::LifecycleNode::SharedPtr parent);

  bool configure();

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_;
};

}  // namespace cs4home_core

#endif  // CS4HOME_CORE__COUPLING_HPP_
