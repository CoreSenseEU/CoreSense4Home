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
#include <utility>
#include <queue>
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

  enum EfferentProcessMode {CALLBACK, ONDEMAND};

  explicit Afferent(rclcpp_lifecycle::LifecycleNode::SharedPtr parent);

  virtual bool configure() = 0;

  void set_mode(
    EfferentProcessMode mode,
    std::function<void(std::unique_ptr<rclcpp::SerializedMessage>)> cb = nullptr);

  EfferentProcessMode get_mode() {return mode_;}
  void set_max_queue_size(size_t size) {max_queue_size_ = size;}
  size_t get_max_queue_size() {return max_queue_size_;}

  template<class MessageT> std::unique_ptr<MessageT> get_msg(
    std::unique_ptr<rclcpp::SerializedMessage> msg)
  {
    auto typed_msg = std::make_unique<MessageT>();
    rclcpp::Serialization<MessageT> serializer;
    serializer.deserialize_message(msg.get(), typed_msg.get());

    return std::move(typed_msg);
  }

  template<class MessageT> std::unique_ptr<MessageT> get_msg()
  {
    if (msg_queue_.empty()) {
      return {};
    }

    std::unique_ptr<rclcpp::SerializedMessage> msg = std::move(msg_queue_.front());
    msg_queue_.pop();

    return get_msg<MessageT>(std::move(msg));
  }

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_;
  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> subs_;

  EfferentProcessMode mode_ {ONDEMAND};

  const size_t MAX_DEFAULT_QUEUE_SIZE = 100;
  size_t max_queue_size_ {MAX_DEFAULT_QUEUE_SIZE};
  std::queue<std::unique_ptr<rclcpp::SerializedMessage>> msg_queue_;

  std::function<void(std::unique_ptr<rclcpp::SerializedMessage>)> callback_;

  bool create_subscriber(const std::string & topic, const std::string & type);
};

}  // namespace cs4home_core

#endif  // CS4HOME_CORE__AFFERENT_HPP_
