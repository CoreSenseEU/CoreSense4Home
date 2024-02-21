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
#include "gtest/gtest.h"

TEST(cognitive_module_test, startup_simple)
{
  rclcpp::init(0, nullptr);

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__node:=cognitive_module_1"});

  auto cm1 = cs4home_core::CognitiveModule::make_shared(options);
  ASSERT_EQ(std::string(cm1->get_name()), "cognitive_module_1");

}
