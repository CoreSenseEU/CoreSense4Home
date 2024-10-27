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


#ifndef CS4HOME_CORE__CORE_HPP_
#define CS4HOME_CORE__CORE_HPP_

#include <memory>


#include "cs4home_core/Afferent.hpp"
#include "cs4home_core/Efferent.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/macros.hpp"

namespace cs4home_core
{

class Core
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Core)

  explicit Core(rclcpp_lifecycle::LifecycleNode::SharedPtr parent);

  virtual bool configure() = 0;
  virtual bool activate() = 0;
  virtual bool deactivate() = 0;

  void set_afferent(cs4home_core::Afferent::SharedPtr afferent) {afferent_ = afferent;}
  void set_efferent(cs4home_core::Efferent::SharedPtr efferent) {efferent_ = efferent;}

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_;
  cs4home_core::Afferent::SharedPtr afferent_;
  cs4home_core::Efferent::SharedPtr efferent_;
};

}  // namespace cs4home_core

#endif  // CS4HOME_CORE__CORE_HPP_
