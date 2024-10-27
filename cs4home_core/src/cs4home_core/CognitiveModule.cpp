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

#include "cs4home_core/CognitiveModule.hpp"

namespace cs4home_core
{

CognitiveModule::CognitiveModule(const rclcpp::NodeOptions & options)
: LifecycleNode("cognitive_module", options)
{
  declare_parameter("core", core_name_);
  declare_parameter("afferent", afferent_name_);
  declare_parameter("efferent", efferent_name_);
  declare_parameter("meta", meta_name_);
  declare_parameter("coupling", coupling_name_);
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT CognitiveModule::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  get_parameter("core", core_name_);
  std::string error_core;
  std::tie(core_, error_core) = load_component<Core>(core_name_, shared_from_this());
  if (core_ == nullptr || !core_->configure()) {
    RCLCPP_ERROR(
      get_logger(), "Error configuring core at %s with name %s: %s",
      get_name(), core_name_.c_str(), error_core.c_str());
    return CallbackReturnT::FAILURE;
  }

  get_parameter("efferent", efferent_name_);
  std::string error_efferent;
  std::tie(efferent_, error_efferent) = load_component<Efferent>(efferent_name_,
    shared_from_this());
  if (efferent_ == nullptr || !efferent_->configure()) {
    RCLCPP_ERROR(
      get_logger(), "Error configuring efferent at %s with name %s: %s",
      get_name(), efferent_name_.c_str(), error_efferent.c_str());
    return CallbackReturnT::FAILURE;
  }

  get_parameter("afferent", afferent_name_);
  std::string error_afferent;
  std::tie(afferent_, error_afferent) = load_component<Afferent>(afferent_name_,
    shared_from_this());
  if (afferent_ == nullptr || !afferent_->configure()) {
    RCLCPP_ERROR(
      get_logger(), "Error configuring afferent at %s with name %s: %s",
      get_name(), afferent_name_.c_str(), error_afferent.c_str());
    return CallbackReturnT::FAILURE;
  }

  core_->set_afferent(afferent_);
  core_->set_efferent(efferent_);

  get_parameter("meta", meta_name_);
  std::string error_meta;
  std::tie(meta_, error_meta) = load_component<Meta>(meta_name_, shared_from_this());
  if (meta_ == nullptr || !meta_->configure()) {
    RCLCPP_ERROR(
      get_logger(), "Error configuring efferent at %s with name %s: %s",
      get_name(), meta_name_.c_str(), error_meta.c_str());
    return CallbackReturnT::FAILURE;
  }

  get_parameter("coupling", coupling_name_);
  std::string error_coupling;
  std::tie(coupling_, error_coupling) = load_component<Coupling>(coupling_name_,
    shared_from_this());
  if (coupling_ == nullptr || !coupling_->configure()) {
    RCLCPP_ERROR(
      get_logger(), "Error configuring efferent at %s with name %s: %s",
      get_name(), coupling_name_.c_str(), error_coupling.c_str());
    return CallbackReturnT::FAILURE;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT CognitiveModule::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  if (!core_->activate()) {
    RCLCPP_ERROR(get_logger(), "Unable to activate Core");
    return CallbackReturnT::FAILURE;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT CognitiveModule::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  if (!core_->deactivate()) {
    RCLCPP_ERROR(get_logger(), "Unable to activate Core");
    return CallbackReturnT::FAILURE;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT CognitiveModule::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT CognitiveModule::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT CognitiveModule::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}


template<class T> std::tuple<typename T::SharedPtr, std::string>
CognitiveModule::load_component(
  const std::string & name, rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
{
  std::string lib_name = "lib" + name + ".so";
  void * handle = dlopen(lib_name.c_str(), RTLD_LAZY);
  if (!handle) {
    return {nullptr, "Cannot open library:" + lib_name};
  }
  using FactoryFunction = typename T::SharedPtr (*)(rclcpp_lifecycle::LifecycleNode::SharedPtr);
  FactoryFunction create_instance = (FactoryFunction)dlsym(handle, "create_instance");
  const char * dlsym_error = dlerror();
  if (dlsym_error) {
    dlclose(handle);
    return {nullptr, std::string("Cannot load symbol 'create_instance': ") + dlsym_error};
  }
  return {create_instance(parent), ""};
}

}  // namespace cs4home_core
