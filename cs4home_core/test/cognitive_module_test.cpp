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


#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "cs4home_core/CognitiveModule.hpp"
#include "gtest/gtest.h"

TEST(cognitive_module_test, startup_simple)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("cs4home_core");
  std::string config_file = pkgpath + "/config/startup_simple.yaml";

  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "-r", "__node:=cognitive_module_1", "--params-file", config_file});

  auto cm1 = cs4home_core::CognitiveModule::make_shared(options);
  ASSERT_EQ(std::string(cm1->get_name()), "cognitive_module_1");

  auto params = cm1->list_parameters({}, 0);
  std::cerr << params.names.size();
  for (const auto & names : params.names) {
    std::cerr << names << std::endl;
  }

  cm1->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(cm1->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
