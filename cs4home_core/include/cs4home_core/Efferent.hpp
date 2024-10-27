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


#ifndef CS4HOME_CORE__EFFERENT_HPP_
#define CS4HOME_CORE__EFFERENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/macros.hpp"

namespace cs4home_core
{

class Efferent
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Efferent)

  explicit Efferent(rclcpp_lifecycle::LifecycleNode::SharedPtr parent);

  virtual bool configure() = 0;

  template<class MessageT>
  void publish(std::unique_ptr<MessageT> msg)
  {
    rclcpp::Serialization<MessageT> serializer;
    auto untyped_msg = rclcpp::SerializedMessage();

    serializer.serialize_message(msg.get(), &untyped_msg);

    for (auto & pub : pubs_) {
      pub->publish(untyped_msg);
    }
  }

protected:
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_;
  std::vector<std::shared_ptr<rclcpp::GenericPublisher>> pubs_;

  bool create_publisher(const std::string & topic, const std::string & type);
};

}  // namespace cs4home_core

#endif  // CS4HOME_CORE__EFFERENT_HPP_
