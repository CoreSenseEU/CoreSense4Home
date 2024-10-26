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


#ifndef CS4HOME_CORE__AFFERENT_HPP_
#define CS4HOME_CORE__AFFERENT_HPP_

#include <memory>
#include <vector>
#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

namespace cs4home_core
{

class Afferent
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Afferent)

  explicit Afferent(rclcpp_lifecycle::LifecycleNode::SharedPtr parent);

  bool configure();

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_;

  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> subs_;

private:
  bool create_subscriber(const std::string & topic, const std::string & type);
};

}  // namespace cs4home_core

#endif  // CS4HOME_CORE__AFFERENT_HPP_
