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

#include "motion/navigation/clear_map_at_goal.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <utility>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace navigation
{

using namespace std::chrono_literals;
using namespace std::placeholders;

ClearMapAtGoal::ClearMapAtGoal(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf),
  costmap_ros_(std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap")),
  radius_(0.0)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  costmap_ros_->on_configure(rclcpp_lifecycle::State());
}

BT::NodeStatus ClearMapAtGoal::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  geometry_msgs::msg::PoseStamped goal;
  getInput("input_goal", goal);
  getInput("radius", radius_);

  RCLCPP_INFO(node_->get_logger(), "Clearing costmap around goal");
  RCLCPP_INFO(node_->get_logger(), "Goal: %f, %f", goal.pose.position.x, goal.pose.position.y);
  RCLCPP_INFO(node_->get_logger(), "Radius: %f", radius_);

  auto layers = costmap_ros_->getLayeredCostmap()->getPlugins();

  RCLCPP_INFO(node_->get_logger(), "Layers retreived");

  RCLCPP_INFO(node_->get_logger(), "before foor loop");
  for (auto & layer : *layers) {
    RCLCPP_INFO(node_->get_logger(), "Trying to clear layer %s", layer->getName().c_str());
    if (layer->isClearable()) {
      RCLCPP_INFO(node_->get_logger(), "Creating layer using static pointer cast");
      auto costmap_layer = std::static_pointer_cast<nav2_costmap_2d::CostmapLayer>(layer);
      RCLCPP_INFO(node_->get_logger(), "Clearing layer");
      try {
        clearLayerRegion(costmap_layer, goal.pose.position.x, goal.pose.position.y, radius_, false);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "Error clearing costmap: %s", e.what());
      }
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Cleared succeeded");
  return BT::NodeStatus::SUCCESS;
}

void ClearMapAtGoal::clearLayerRegion(
  std::shared_ptr<nav2_costmap_2d::CostmapLayer> & costmap, double pose_x, double pose_y,
  double reset_distance, bool invert)
{
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  double start_point_x = pose_x - reset_distance / 2;
  double start_point_y = pose_y - reset_distance / 2;
  double end_point_x = start_point_x + reset_distance;
  double end_point_y = start_point_y + reset_distance;

  int start_x, start_y, end_x, end_y;
  costmap->worldToMapEnforceBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapEnforceBounds(end_point_x, end_point_y, end_x, end_y);

  costmap->clearArea(start_x, start_y, end_x, end_y, invert);

  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
}

}  // namespace navigation

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<navigation::ClearMapAtGoal>(name, config);
    };

  factory.registerBuilder<navigation::ClearMapAtGoal>("ClearMapAtGoal", builder);
  // factory.registerNodeType<navigation::ClearMapAtGoal>("ClearMapAtGoal");
}
