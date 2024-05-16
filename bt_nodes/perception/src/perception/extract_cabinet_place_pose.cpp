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

#include "perception/extract_cabinet_place_pose.hpp"

#include <string>
#include <utility>

#include "perception_system_interfaces/srv/isolate_pc_background.hpp"

namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

ExtractCabinetPlacePose::ExtractCabinetPlacePose(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: perception::BtServiceNode<perception_system_interfaces::srv::IsolatePCBackground>(
    xml_tag_name, action_name, conf)
{
}

void ExtractCabinetPlacePose::on_tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "ExtractCabinetPlacePose ticked");
  getInput("selected_object", selected_object_);
  config().blackboard->get("tf_buffer", tf_buffer_);
  

  request_->classes = {};
}

void ExtractCabinetPlacePose::on_result()
{
  if (result_.success) {
    if (result_.filtered_pc.data.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Empty pointcloud");
      setStatus(BT::NodeStatus::FAILURE);
      return;
    }
    // if (result_.filtered_pc->header.frame_id != selected_object_->header.frame_id) {
    //  TODO(implement this)


    // }

    setStatus(BT::NodeStatus::SUCCESS);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(result_.filtered_pc, *cloud_);
    crop_.setInputCloud(cloud_);

    crop_.setMin(Eigen::Vector4f(-1.0,
                                -0.2,
                                selected_object_->pose.position.z,
                                0.));
    crop_.setMax(Eigen::Vector4f(+1.0,
                                +0.2,
                                selected_object_->pose.position.z + 0.2,
                                0.));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cabinet(new pcl::PointCloud<pcl::PointXYZRGB>());
    crop_.filter(*segmented_cabinet);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*segmented_cabinet, centroid);
    geometry_msgs::msg::PoseStamped place_pose;
    place_pose.header.frame_id = result_.filtered_pc.header.frame_id;
    place_pose.pose.position.x = centroid[0];
    place_pose.pose.position.y = centroid[1];
    place_pose.pose.position.z = centroid[2];
    place_pose.pose.orientation.w = 1.0;
    setOutput("place_pose", place_pose);
    
  } else {
    std::cout << "Failure" << std::endl;
    // setOutput("listen_text", result_.result->text);
    setStatus(BT::NodeStatus::FAILURE);
  }
}

}  // namespace perception

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<perception::ExtractCabinetPlacePose>(
        name, "/isolate_pc_background", config);
    };

  factory.registerBuilder<perception::ExtractCabinetPlacePose>("ExtractCabinetPlacePose", builder);
}
