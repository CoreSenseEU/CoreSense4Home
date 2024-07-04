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
// See the License for the specific language governing permissions andGO2OBJECT
// limitations under the License.

#ifndef HRI__START_MUSIC_HPP_
#define HRI__START_MUSIC_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "hri/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"
#include "audio_common_msgs/srv/music_play.hpp"


namespace hri
{

class StartMusic
  : public hri::BtServiceNode<yolov8_msgs::srv::ChangeModel,
  rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit StartMusic(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  void on_result() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({
      BT::InputPort<std::string>("audio", "elevator", "audio to play"),
    });
  }

private:
  std::string audio_;
};

}  // namespace hri

#endif  // HRI__START_MUSIC_HPP_
