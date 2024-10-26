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


#ifndef CS4HOME_CORE__MACROS_HPP_
#define CS4HOME_CORE__MACROS_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

namespace cs4home_core
{

#define CS_REGISTER_COMPONENT(class_name) \
  extern "C" class_name::SharedPtr create_instance( \
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent) \
  { \
    return class_name::make_shared(parent); \
  }

}  // namespace cs4home_core

#endif  // CS4HOME_CORE__MACROS_HPP_
