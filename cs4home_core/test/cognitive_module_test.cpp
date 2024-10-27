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

#include <optional>

#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "cs4home_core/Afferent.hpp"
#include "cs4home_core/CognitiveModule.hpp"
#include "gtest/gtest.h"


template<class T> std::tuple<typename T::SharedPtr, std::string>
load_component(
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

using namespace std::chrono_literals;

TEST(cognitive_module_test, afferent_on_demand)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lc_node");
  auto pub_node = rclcpp::Node::make_shared("pub_node");
  auto pub = pub_node->create_publisher<sensor_msgs::msg::Image>("/image", 100);

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_node(pub_node);

  std::vector<std::string> topics {"/image"};

  auto [afferent, error_afferent] = load_component<cs4home_core::Afferent>(
    "simple_image_input", node);
  ASSERT_NE(afferent, nullptr);

  node->set_parameter(rclcpp::Parameter("simple_image_input.topics", topics));
  ASSERT_TRUE(afferent->configure());

  sensor_msgs::msg::Image msg;
  for (int i = 0; i < 10; i++) {
    msg.header.frame_id = std::to_string(i);
    pub->publish(msg);
    exe.spin_some();
  }

  auto start = node->now();
  while (node->now() - start < 1s) {
    exe.spin_some();
  }

  for (int i = 0; i < 10; i++) {
    auto in_msg = afferent->get_msg<sensor_msgs::msg::Image>();
    ASSERT_NE(in_msg, nullptr);
    ASSERT_EQ(i, std::atoi(in_msg->header.frame_id.c_str()));
  }

  auto in_msg = afferent->get_msg<sensor_msgs::msg::Image>();
  ASSERT_EQ(in_msg, nullptr);
}


TEST(cognitive_module_test, afferent_on_subscription)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lc_node");
  auto pub_node = rclcpp::Node::make_shared("pub_node");
  auto pub = pub_node->create_publisher<sensor_msgs::msg::Image>("/image", 100);

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_node(pub_node);

  std::vector<std::string> topics {"/image"};
  std::vector<std::unique_ptr<rclcpp::SerializedMessage>> images;

  auto [afferent, error_afferent] = load_component<cs4home_core::Afferent>(
    "simple_image_input", node);
  ASSERT_NE(afferent, nullptr);

  node->set_parameter(rclcpp::Parameter("simple_image_input.topics", topics));
  afferent->set_mode(cs4home_core::Afferent::CALLBACK);
  ASSERT_EQ(afferent->get_mode(), cs4home_core::Afferent::ONDEMAND);

  afferent->set_mode(cs4home_core::Afferent::CALLBACK,
    [&images](std::unique_ptr<rclcpp::SerializedMessage> msg) {
      images.push_back(std::move(msg));
    }
  );
  ASSERT_EQ(afferent->get_mode(), cs4home_core::Afferent::CALLBACK);
  ASSERT_TRUE(afferent->configure());

  sensor_msgs::msg::Image msg;
  for (int i = 0; i < 10; i++) {
    msg.header.frame_id = std::to_string(i);
    pub->publish(msg);
    exe.spin_some();
  }

  auto start = node->now();
  while (node->now() - start < 1s) {
    exe.spin_some();
  }

  ASSERT_EQ(images.size(), 10);
  for (int i = 0; i < 10; i++) {
    std::unique_ptr<rclcpp::SerializedMessage> in_msg = std::move(images[i]);
    ASSERT_NE(in_msg, nullptr);

    rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
    auto typed_msg = std::make_unique<sensor_msgs::msg::Image>();
    serializer.deserialize_message(in_msg.get(), typed_msg.get());

    ASSERT_EQ(i, std::atoi(typed_msg->header.frame_id.c_str()));
  }
}


TEST(cognitive_module_test, efferent)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lc_node");
  auto sub_node = rclcpp::Node::make_shared("sub_node");

  std::vector<sensor_msgs::msg::Image> images;
  auto sub = sub_node->create_subscription<sensor_msgs::msg::Image>(
    "/image", 100, [&images] (sensor_msgs::msg::Image msg) {
      images.push_back(msg);
    });

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_node(sub_node);

  std::vector<std::string> topics {"/image"};

  auto [efferent, error_efferent] = load_component<cs4home_core::Efferent>(
    "simple_image_output", node);
  ASSERT_NE(efferent, nullptr);

  node->set_parameter(rclcpp::Parameter("simple_image_output.topics", topics));
  ASSERT_TRUE(efferent->configure());

  for (int i = 0; i < 10; i++) {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.frame_id = std::to_string(i);
    efferent->publish(std::move(msg));
    exe.spin_some();
  }

  auto start = node->now();
  while (node->now() - start < 1s) {
    exe.spin_some();
  }

  ASSERT_EQ(images.size(), 10);
  for (int i = 0; i < 10; i++) {
    ASSERT_EQ(i, std::atoi(images[i].header.frame_id.c_str()));
  }
}


TEST(cognitive_module_test, core)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lc_node");
  auto pub_node = rclcpp::Node::make_shared("pub_node");
  auto sub_node = rclcpp::Node::make_shared("sub_node");

  auto pub = pub_node->create_publisher<sensor_msgs::msg::Image>("/in_image", 100);

  std::vector<sensor_msgs::msg::Image> images;
  auto sub = sub_node->create_subscription<sensor_msgs::msg::Image>(
    "/out_image", 100, [&images] (sensor_msgs::msg::Image msg) {
      images.push_back(msg);
    });

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_node(pub_node);
  exe.add_node(sub_node);

  std::vector<std::string> in_topics {"/in_image"};
  std::vector<std::string> out_topics {"/out_image"};

  auto [afferent, error_afferent] = load_component<cs4home_core::Afferent>(
    "simple_image_input", node);
  ASSERT_NE(afferent, nullptr);
  auto [efferent, error_efferent] = load_component<cs4home_core::Efferent>(
    "simple_image_output", node);
  ASSERT_NE(efferent, nullptr);
  auto [core, error_core] = load_component<cs4home_core::Core>(
    "image_filter", node);
  ASSERT_NE(core, nullptr);

  node->set_parameter(rclcpp::Parameter("simple_image_input.topics", in_topics));
  node->set_parameter(rclcpp::Parameter("simple_image_output.topics", out_topics));
  ASSERT_TRUE(afferent->configure());
  ASSERT_TRUE(efferent->configure());
  core->set_afferent(afferent);
  core->set_efferent(efferent);
  ASSERT_TRUE(core->configure());
  ASSERT_TRUE(core->activate());

  sensor_msgs::msg::Image msg;
  for (int i = 0; i < 10; i++) {
    msg.header.frame_id = std::to_string(i);
    pub->publish(msg);
    exe.spin_some();
  }

  auto start = node->now();
  while (node->now() - start < 1s) {
    exe.spin_some();
  }

  ASSERT_TRUE(core->deactivate());

  ASSERT_EQ(images.size(), 10);
  for (int i = 0; i < 10; i++) {
    ASSERT_EQ(i * 2, std::atoi(images[i].header.frame_id.c_str()));
  }
}

TEST(cognitive_module_test, core_cb)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_lc_node");
  auto pub_node = rclcpp::Node::make_shared("pub_node");
  auto sub_node = rclcpp::Node::make_shared("sub_node");

  auto pub = pub_node->create_publisher<sensor_msgs::msg::Image>("/in_image", 100);

  std::vector<sensor_msgs::msg::Image> images;
  auto sub = sub_node->create_subscription<sensor_msgs::msg::Image>(
    "/out_image", 100, [&images] (sensor_msgs::msg::Image msg) {
      images.push_back(msg);
    });

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_node(pub_node);
  exe.add_node(sub_node);

  std::vector<std::string> in_topics {"/in_image"};
  std::vector<std::string> out_topics {"/out_image"};

  auto [afferent, error_afferent] = load_component<cs4home_core::Afferent>(
    "simple_image_input", node);
  ASSERT_NE(afferent, nullptr);
  auto [efferent, error_efferent] = load_component<cs4home_core::Efferent>(
    "simple_image_output", node);
  ASSERT_NE(efferent, nullptr);
  auto [core, error_core] = load_component<cs4home_core::Core>(
    "image_filter_cb", node);
  ASSERT_NE(core, nullptr);

  node->set_parameter(rclcpp::Parameter("simple_image_input.topics", in_topics));
  node->set_parameter(rclcpp::Parameter("simple_image_output.topics", out_topics));
  ASSERT_TRUE(afferent->configure());
  ASSERT_TRUE(efferent->configure());
  core->set_afferent(afferent);
  core->set_efferent(efferent);
  ASSERT_TRUE(core->configure());
  ASSERT_TRUE(core->activate());

  sensor_msgs::msg::Image msg;
  for (int i = 0; i < 10; i++) {
    msg.header.frame_id = std::to_string(i);
    pub->publish(msg);
    exe.spin_some();
  }

  auto start = node->now();
  while (node->now() - start < 1s) {
    exe.spin_some();
  }

  ASSERT_TRUE(core->deactivate());

  ASSERT_EQ(images.size(), 10);
  for (int i = 0; i < 10; i++) {
    ASSERT_EQ(i * 2, std::atoi(images[i].header.frame_id.c_str()));
  }
}


TEST(cognitive_module_test, startup_simple)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("cs4home_core");
  std::string config_file = pkgpath + "/config/startup_simple_1.yaml";

  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "-r", "__node:=cognitive_module_1", "--params-file", config_file});

  auto cm1 = cs4home_core::CognitiveModule::make_shared(options);
  ASSERT_EQ(std::string(cm1->get_name()), "cognitive_module_1");

  auto params = cm1->list_parameters({}, 0);
  ASSERT_EQ(params.names.size(), 7u);

  auto pub_node = rclcpp::Node::make_shared("pub_node");
  auto sub_node = rclcpp::Node::make_shared("sub_node");

  auto pub = pub_node->create_publisher<sensor_msgs::msg::Image>("/image_raw", 100);

  std::vector<sensor_msgs::msg::Image> images;
  auto sub = sub_node->create_subscription<sensor_msgs::msg::Image>(
    "/detections", 100, [&images] (sensor_msgs::msg::Image msg) {
      images.push_back(msg);
    });

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(cm1->get_node_base_interface());
  exe.add_node(pub_node);
  exe.add_node(sub_node);

  cm1->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(cm1->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  cm1->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(cm1->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  sensor_msgs::msg::Image msg;
  for (int i = 0; i < 10; i++) {
    msg.header.frame_id = std::to_string(i);
    pub->publish(msg);
    exe.spin_some();
  }

  auto start = cm1->now();
  while (cm1->now() - start < 1s) {
    exe.spin_some();
  }

  cm1->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  ASSERT_EQ(cm1->get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  ASSERT_EQ(images.size(), 10);
  for (int i = 0; i < 10; i++) {
    ASSERT_EQ(i * 2, std::atoi(images[i].header.frame_id.c_str()));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
