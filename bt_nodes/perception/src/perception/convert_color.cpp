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

#include "perception/convert_color.hpp"

#include <limits>
#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "perception_system/PerceptionUtils.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

ConvertColor::ConvertColor(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  getInput("interest", interest_); // top, bottom
  getInput("color", color_);
}

void ConvertColor::halt()
{
  RCLCPP_DEBUG(node_->get_logger(), "ConvertColor halted");
}

BT::NodeStatus ConvertColor::tick()
{

  try {

    cv::Scalar rgbColorUp = getColorRGB(color_);

    // Convert RGB to HSV for the upper part
    cv::Mat rgbMatUp(1, 1, CV_8UC3, rgbColorUp);
    cv::Mat hsvMatUp;
    cv::cvtColor(rgbMatUp, hsvMatUp, cv::COLOR_BGR2HSV);

    cv::Scalar hsvUp = cv::Scalar(0, 0, 0);
    cv::Scalar hsvDown = cv::Scalar(0, 0, 0);

    if (interest_ == "top") {
      hsvUp = hsvMatUp.at<cv::Vec3b>(0, 0);

    } else if (interest_ == "bottom") {
      hsvDown = hsvMatUp.at<cv::Vec3b>(0, 0);
    }

    int64_t person_id_ = calculatePersonID(hsvUp, hsvDown);

    setOutput("person_id", person_id_);
    RCLCPP_INFO(
      node_->get_logger(), "[ConvertColor] Person id: %ld",
      person_id_);

    return BT::NodeStatus::SUCCESS;

  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid color");
    return BT::NodeStatus::FAILURE;
  }
}

cv::Scalar ConvertColor::getColorRGB(const std::string & colorName)
{

  static const std::unordered_map<std::string, cv::Scalar> colorMap = {
    {"red", cv::Scalar(0, 0, 255)},
    {"green", cv::Scalar(0, 255, 0)},
    {"blue", cv::Scalar(255, 0, 0)},
    {"yellow", cv::Scalar(0, 255, 255)},
    {"white", cv::Scalar(255, 255, 255)},
    {"black", cv::Scalar(0, 0, 0)},
    {"purple", cv::Scalar(128, 0, 128)},
    {"orange", cv::Scalar(0, 165, 255)},
    {"gray", cv::Scalar(128, 128, 128)},
    {"pink", cv::Scalar(255, 192, 203)}};

  auto it = colorMap.find(colorName);

  if (it != colorMap.end()) {
    return it->second;
    RCLCPP_DEBUG(node_->get_logger(), "[ConvertColor] Color found");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[ConvertColor] Color not found");
    throw std::invalid_argument("Color invalid");
  }
}

int64_t ConvertColor::calculatePersonID(
  const cv::Scalar & hsv_up,
  const cv::Scalar & hsv_down)
{

  int64_t h01_up = static_cast<int>(hsv_up[0] / 180.0 * 100);
  int64_t s01_up = static_cast<int>(hsv_up[1] / 255.0 * 100);
  int64_t v01_up = static_cast<int>(hsv_up[2] / 255.0 * 100);

  int64_t h01_down = static_cast<int>(hsv_down[0] / 180.0 * 100);
  int64_t s01_down = static_cast<int>(hsv_down[1] / 255.0 * 100);
  int64_t v01_down = static_cast<int>(hsv_down[2] / 255.0 * 100);

  int64_t result = h01_up * static_cast<int64_t>(std::pow(10, 16)) +
    s01_up * static_cast<int64_t>(std::pow(10, 14)) +
    v01_up * static_cast<int64_t>(std::pow(10, 12)) +
    h01_down * static_cast<int64_t>(std::pow(10, 10)) +
    s01_down * static_cast<int64_t>(std::pow(10, 8)) +
    v01_down * static_cast<int64_t>(std::pow(10, 6));
  return result;
}
} // namespace perception

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<perception::ConvertColor>("ConvertColor");
}
