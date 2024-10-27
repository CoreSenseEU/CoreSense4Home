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


#include "cs4home_core/Meta.hpp"
#include "cs4home_core/macros.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/macros.hpp"

class DefaultMeta : public cs4home_core::Meta
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(DefaultMeta)

  explicit DefaultMeta(rclcpp_lifecycle::LifecycleNode::SharedPtr parent)
  : Meta(parent)
  {
    RCLCPP_DEBUG(parent_->get_logger(), "Meta created: [DefaultMeta]");
  }


  bool configure()
  {
    RCLCPP_DEBUG(parent_->get_logger(), "Meta configured");
    return true;
  }
};

CS_REGISTER_COMPONENT(DefaultMeta)
