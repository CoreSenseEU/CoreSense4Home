// Copyright 2021 Intelligent Robotics Lab
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

#include <string>

#include "attention_system/AttentionServerNode.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "perception_system/ObjectsDetectionNode.hpp"
#include "perception_system/PeopleDetectionNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::ExecutorOptions exe_rt_options;
  rclcpp::executors::MultiThreadedExecutor executor_rt(exe_rt_options, 4);

  rclcpp::NodeOptions node_options;
  // node_options.use_intra_process_comms(true);

  auto aux_node = rclcpp::Node::make_shared("systems_bringup");

  // Creating of all systems nodes
  std::list<std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode>> sched_nodes = {
    std::make_shared<attention_system::AttentionServerNode>(node_options),
    // std::make_shared<perception_system::PeopleDetectionNode>(node_options),
    std::make_shared<perception_system::ObjectsDetectionNode>(node_options)
  };

  // Adding systems nodes to the appropiate executor
  for (auto & sched_node : sched_nodes) {
    executor_rt.add_node(sched_node->get_node_base_interface());
  }

  // Change systems nodes state to Configure
  RCLCPP_INFO(aux_node->get_logger(), "Configuring systems nodes");
  for (auto & sched_node : sched_nodes) {
    sched_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  }
  RCLCPP_INFO(aux_node->get_logger(), "Finished configuring systems nodes");

  // Spinnning systems nodes in their executors
  RCLCPP_INFO(aux_node->get_logger(), "Executing Systems");
  auto realtime_thread = std::thread(
    [&]() {
      sched_param sch;
      sch.sched_priority = 60;
      if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) == -1) {
        perror("pthread_setschedparam failed");
        exit(-1);
      }
      executor_rt.spin();
    });

  rclcpp::spin(aux_node);

  // Clean exit
  RCLCPP_INFO(aux_node->get_logger(), "Finished executing Systems - waiting for clean exit");
  realtime_thread.join();

  rclcpp::shutdown();
  RCLCPP_INFO(aux_node->get_logger(), "Finished executing Systems - exiting");

  return 0;
}
