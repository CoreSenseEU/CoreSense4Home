// Copyright (c) 2018 Intel Corporation
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

#ifndef BTSERVICENODE_HPP_
#define BTSERVICENODE_HPP_

#include <memory>
#include <rclcpp/allocator/allocator_common.hpp>
#include <rclcpp/executors.hpp>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace robocup_hri
{

using namespace std::chrono_literals;  // NOLINT

template<class ServiceT, class NodeT = rclcpp::Node>
class BtServiceNode : public BT::ActionNodeBase
{
public:
  BtServiceNode(
    const std::string & xml_tag_name, const std::string & service_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), service_name_(service_name)
  {
    node_ = config().blackboard->get<typename NodeT::SharedPtr>("node");

    server_timeout_ = 1s;

    // Initialize the input and output messages
    request_ = std::make_shared<typename ServiceT::Request>();
    result_ = typename ServiceT::Response();

    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      service_name_ = remapped_action_name;
    }
    createServiceClient(service_name_);

    // Give the derive class a chance to do any initialization
    RCLCPP_INFO(node_->get_logger(), "\"%s\" BtServiceNode initialized", xml_tag_name.c_str());
  }

  BtServiceNode() = delete;

  virtual ~BtServiceNode() {}

  // Create instance of an action server
  void createServiceClient(const std::string & service_name)
  {
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    // Now that we have the ROS node to use, create the action client for this BT action
    service_client_ = node_->template create_client<ServiceT>(
      service_name, rmw_qos_profile_services_default, callback_group_);

    // Make sure the server is actually there before continuing
    RCLCPP_INFO(node_->get_logger(), "Waiting for \"%s\" service server", service_name.c_str());
    service_client_->wait_for_service();
  }

  // Any subclass of BtServiceNode that accepts parameters must provide a providedPorts method
  // and call providedBasicPorts in it.
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")};
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  static BT::PortsList providedPorts() {return providedBasicPorts({});}

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, on_wait_for_result, and on_success

  // Could do dynamic checks, such as getting updates to values on the blackboard
  virtual void on_tick() {}

  /** Callback invoked when the response is received by the server.
   * It is up to the user to define if this returns SUCCESS or FAILURE.
   */
  virtual void on_result() {}

  // The main override required by a BT action
  BT::NodeStatus tick() override
  {
    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE || !service_client_) {
      createServiceClient(service_name_);

      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);

      // user defined callback
      on_tick();

      on_new_request_received();
    }

    // The following code corresponds to the "RUNNING" loop
    if (rclcpp::ok() && !request_result_available_) {
      // user defined callback. May modify the value of "goal_updated_"

      if (request_updated_) {
        request_updated_ = false;
        on_new_request_received();
      }

      // rclcpp::spin_some(node_->get_node_base_interface());

      // check if, after invoking spin_some(), we finally received the result
      if (!request_result_available_) {
        // Yield this Action, returning RUNNING
        return BT::NodeStatus::RUNNING;
      }
    }

    on_result();

    return status();
  }

  // The other (optional) override required by a BT action. In this case, we
  // make sure to cancel the ROS2 action if it is still running.
  void halt() override {setStatus(BT::NodeStatus::IDLE);}

protected:
  void on_new_request_received()
  {
    request_result_available_ = false;

    // auto result_callback =
    //     [&,this](typename rclcpp::Client<ServiceT>::SharedFuture future_result) {
    //         request_result_available_ = true;
    //         result_ = *future_result.get();
    // };

    auto future_request = service_client_->async_send_request(request_).share();

    auto ret = callback_group_executor_.spin_until_future_complete(
      future_request);

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("send_request failed");
    } else {
      request_result_available_ = true;
      result_ = *future_request.get();
      future_request = {};
    }
  }

  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  std::string service_name_;
  typename std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;

  // All ROS2 actions have a goal and a result
  typename ServiceT::Request::SharedPtr request_;
  bool request_updated_{false};
  bool request_result_available_{false};
  typename ServiceT::Response result_;

  // The node that will be used for any ROS operations
  typename NodeT::SharedPtr node_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while waiting for response from a server when a
  // new action goal is sent or canceled
  std::chrono::milliseconds server_timeout_;
};

}  // namespace robocup_hri

#endif  // BTSERVICENODE_HPP_
