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

#include "motion/navigation/configure_navigate_back.hpp"

namespace navigation
{

using namespace std::chrono_literals;
using namespace std::placeholders;

ConfigureNavigateBack::ConfigureNavigateBack(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  mode_(navigation_system_interfaces::msg::Mode::AMCL),
  qos_(rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable())
{
  config().blackboard->get("node", node_);
  map_path_ = ament_index_cpp::get_package_share_directory("robocup_bringup") + "/maps/carry_mapped_arena";
  save_map_client_ = node_->create_client<slam_toolbox::srv::SaveMap>("slam_toolbox/save_map");
  set_mode_client_ = node_->create_client<navigation_system_interfaces::srv::SetMode>(
    "navigation_system_node/set_mode");
  load_map_client_ = node_->create_client<navigation_system_interfaces::srv::SetMap>(
    "navigation_system_node/set_map");

  intial_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

  // sub_pose_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    // "amcl_pose", qos_, std::bind(&ConfigureNavigateBack::pose_callback, this, _1));
  sub_initial_pose_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", qos_, std::bind(&ConfigureNavigateBack::initial_pose_callback, this, _1));

}

BT::NodeStatus ConfigureNavigateBack::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "ConfigureNavigateBack tick");

  rclcpp::spin_some(node_->get_node_base_interface());

  if (status() == BT::NodeStatus::IDLE)
  {
    on_idle();
  }
  try {
    entity_transform_ = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not get transform from map to base_footprint: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
  current_pos_.header.stamp = entity_transform_.header.stamp;
  current_pos_.header.frame_id = "map";
  current_pos_.pose.pose.position.x = entity_transform_.transform.translation.x;
  current_pos_.pose.pose.position.y = entity_transform_.transform.translation.y;

  current_pos_.pose.pose.orientation = entity_transform_.transform.rotation;
  

  
  
  bool success = true;
  
  if (!is_save_map_)
  {
    is_save_map_ = save_map(map_path_);
  }
  if (!is_set_mode_ && is_save_map_)
  {
    is_set_mode_ = set_nav2_mode(mode_);
  }
  if (!is_set_map_ && is_set_mode_ && is_save_map_)
  {
    is_set_map_ = set_nav2_map(map_path_+ ".yaml");
  }
  if (!is_initial_pose_ && is_set_map_ && is_set_mode_ && is_save_map_)
  {
    is_initial_pose_ = set_initial_pose(current_pos_);
    if (is_initial_pose_)
    {
      success = true;
    }   
  }

  if (success)
  {
    RCLCPP_INFO(node_->get_logger(), "ConfigureNavigateBack finished SUCCESS!!");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }

}

void ConfigureNavigateBack::initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "Initial pose setted!");
  is_initial_pose_received_ = true;
}

// void ConfigureNavigateBack::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
// {
//   current_pos_ = *msg;
// }

void ConfigureNavigateBack::halt() {RCLCPP_INFO(node_->get_logger(), "ConfigureNavigateBack halted");}  
bool ConfigureNavigateBack::save_map(const std::string & map_name)
{
  auto request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
  request->name.data = map_name;

  auto result = save_map_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node_->get_logger(), "Map saved!!");
    return true;
  }
  else
  {
    return false;
  }
}

bool ConfigureNavigateBack::set_nav2_mode(const int8_t & mode)
{
  auto request = std::make_shared<navigation_system_interfaces::srv::SetMode::Request>();
  request->mode.id = mode;

  auto future = set_mode_client_->async_send_request(request).share();
  if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = *future.get();
    if (!result.success)
    {
      RCLCPP_INFO(node_->get_logger(), "Mode setted!!");
      return false;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Mode setted!!");
      return true;
    }
  }
  else
  {
    return false;
  }
}
bool ConfigureNavigateBack::set_nav2_map(const std::string & map_path)
{
  RCLCPP_INFO(node_->get_logger(), "Setting map...");
  auto request = std::make_shared<navigation_system_interfaces::srv::SetMap::Request>();
  request->map_path = map_path;

  auto future = load_map_client_->async_send_request(request).share();
  if (rclcpp::spin_until_future_complete(node_, future, 5s) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = *future.get();
    if (!result.success)
    {
      RCLCPP_INFO(node_->get_logger(), "Coudnt set the map an error ocurred!");
      return false;
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Map setted!!");
      return true;
    }
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Coudnt set the map an error ocurred with the future!");
    return false;
  }
}

BT::NodeStatus ConfigureNavigateBack::on_idle()
{
  RCLCPP_INFO(node_->get_logger(), "ConfigureNavigateBack on_idle");
  config().blackboard->get("tf_buffer", tf_buffer_);
  return BT::NodeStatus::RUNNING;
}

bool ConfigureNavigateBack::set_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  RCLCPP_INFO(node_->get_logger(), "Setting initial pose...");
  
  auto start_time = node_->now();
  auto rate = rclcpp::Rate(0.5);
  while (rclcpp::ok() && !is_initial_pose_received_ && (node_->now() - start_time) < 5s)
  {
    RCLCPP_INFO(node_->get_logger(), "spinning !!");
    rclcpp::spin_some(node_->get_node_base_interface());
    intial_pose_pub_->publish(pose);
    rate.sleep();
  }
  if (is_initial_pose_received_)
  {
    RCLCPP_INFO(node_->get_logger(), "Initial pose setted!!");
    return true;
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Initial pose not setted!!");
    return false;
  }
}

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<navigation::ConfigureNavigateBack>("ConfigureNavigateBack");
}

}  // namespace navigation