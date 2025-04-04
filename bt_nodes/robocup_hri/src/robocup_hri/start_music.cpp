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

#include "robocup_hri/start_music.hpp"

#include <string>

namespace robocup_hri
{

using namespace std::chrono_literals;
using namespace std::placeholders;

StartMusic::StartMusic(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: robocup_hri::BtServiceNode<audio_common_msgs::srv::MusicPlay,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf)
{
}

void StartMusic::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "StartMusic ticked");

  getInput("audio", audio_);
  getInput("loop", loop_);

  request_->audio = audio_;
  request_->loop = loop_;
}

void StartMusic::on_result()
{
  if (result_.success) {
    std::cout << "Success StartMusic" << std::endl;
    setStatus(BT::NodeStatus::SUCCESS);

  } else {
    std::cout << "Failure StartMusic" << std::endl;
    // setOutput("listen_text", result_.result->text);
    setStatus(BT::NodeStatus::FAILURE);
  }
}

}  // namespace perception

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<robocup_hri::StartMusic>(
        name, "/music_play", config);
    };

  factory.registerBuilder<robocup_hri::StartMusic>("StartMusic", builder);
}
