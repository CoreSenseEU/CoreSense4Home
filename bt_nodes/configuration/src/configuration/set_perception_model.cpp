// Copyright 2024 Intelligent Robotics Lab - Gentlebots
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

#include "configuration/set_perception_model.hpp"

#include <string>
#include <utility>

namespace configuration
{

using namespace std::chrono_literals;
using namespace std::placeholders;

SetPerceptionModel::SetPerceptionModel(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: configuration::BtServiceNode<yolov8_msgs::srv::ChangeModel,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
}

constexpr unsigned int str2int(const char * str, int h = 0)
{
  return !str[h] ? 5381 : (str2int(str, h + 1) * 33) ^ str[h];
}

inline std::string joinPaths(const std::string & path1, const std::string & path2)
{
  if (path1.back() == '/') {
    return path1 + path2;
  } else {
    return path1 + "/" + path2;      // Adjust the separator based on your OS.
  }
}

void SetPerceptionModel::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "SetPerceptionModelNode ticked");
  getInput("model_name", model_name_);
  getInput("model_type", model_type_);
  getInput("model_path", model_path_);

  if (!model_name_.empty()) {
    switch (str2int(model_type_.c_str())) {
      case str2int("object"):
      default:
        model_name_ = "yolov8m.pt";
        break;
      case str2int("keypoint"):
        model_name_ = "yolov8m-pose.pt";
        break;
      case str2int("segmentation"):
        model_name_ = "yolov8m-seg.pt";
        break;
    }
  }

  auto model_path = joinPaths(model_path_, model_name_);

  request_->model = model_path;
}

void SetPerceptionModel::on_result()
{
  if (result_.success) {
    std::cout << "Success SetPerceptionNode" << std::endl;
    setStatus(BT::NodeStatus::SUCCESS);

  } else {
    std::cout << "Failure SetPerceptionNode" << std::endl;
    // setOutput("listen_text", result_.result->text);
    setStatus(BT::NodeStatus::FAILURE);
  }
}

}  // namespace perception

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<configuration::SetPerceptionModel>(
        name, "/change_model", config);
    };

  factory.registerBuilder<configuration::SetPerceptionModel>("SetPerceptionModel", builder);
}
