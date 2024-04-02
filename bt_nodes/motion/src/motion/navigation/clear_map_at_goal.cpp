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

#include <string>
#include <utility>


#include "motion/navigation/clear_map_at_goal.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "behaviortree_cpp_v3/behavior_tree.h"


namespace navigation
{

using namespace std::chrono_literals;
using namespace std::placeholders;


ClearMapAtGoal::ClearMapAtGoal(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf),
costmap_ros_(std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap_bt")),
radius_(0.0)
{
  // config().blackboard->get("node", node_);
}

BT::NodeStatus
ClearMapAtGoal::tick()
{

  geometry_msgs::msg::PoseStamped goal;
  getInput("input_goal", goal);
  getInput("radius", radius_);

  auto layers = costmap_ros_->getLayeredCostmap()->getPlugins();

  for (auto & layer : *layers) {
    if (layer->isClearable()) {
      auto costmap_layer = std::static_pointer_cast<nav2_costmap_2d::CostmapLayer>(layer);
      clearLayerRegion(costmap_layer, goal.pose.position.x, goal.pose.position.y, radius_, false);
    }
  }

  return BT::NodeStatus::SUCCESS;
}

void clearLayerRegion(
  std::shared_ptr<nav2_costmap_2d::CostmapLayer> & costmap, double pose_x, double pose_y, double reset_distance,
  bool invert)
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
  factory.registerNodeType<navigation::ClearMapAtGoal>("ClearMapAtGoal");
}
