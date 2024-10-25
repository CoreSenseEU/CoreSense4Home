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


#ifndef CS4HOME_CORE__COGNITIVEMODULE_HPP_
#define CS4HOME_CORE__COGNITIVEMODULE_HPP_

#include <dlfcn.h>

#include <tuple>
#include <string>

#include "cs4home_core/Afferent.hpp"
#include "cs4home_core/Core.hpp"
#include "cs4home_core/Coupling.hpp"
#include "cs4home_core/Efferent.hpp"
#include "cs4home_core/Meta.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace cs4home_core
{

class CognitiveModule : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(CognitiveModule)
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit CognitiveModule(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

protected:
  Afferent::SharedPtr afferent_;
  Efferent::SharedPtr efferent_;
  Core::SharedPtr core_;
  Meta::SharedPtr meta_;
  Coupling::SharedPtr coupling_;

  std::string core_name_;
  std::string afferent_name_;
  std::string efferent_name_;
  std::string meta_name_;
  std::string coupling_name_;

  template<class T>
  std::tuple<typename T::SharedPtr, std::string> load_component(
    const std::string & name, rclcpp_lifecycle::LifecycleNode::SharedPtr parent);
};

}  // namespace cs4home_core

#endif  // CS4HOME_CORE__COGNITIVEMODULE_HPP_

// #include "rclcpp_components/register_node_macro.hpp"
//
// // Register the component with class_loader.
// // This acts as a sort of entry point, allowing the component to be discoverable when its library
// // is being loaded into a running process.
// RCLCPP_COMPONENTS_REGISTER_NODE(cs4home_core::CognitiveModule)
