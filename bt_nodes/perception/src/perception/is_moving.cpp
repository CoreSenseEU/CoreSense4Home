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
#include <limits>

#include "perception/is_moving.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "perception_system/PerceptionUtils.hpp"


namespace perception
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsMoving::IsMoving(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  config().blackboard->get("cam_frame", cam_frame_);

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


  getInput("frame", frame_);
  getInput("velocity_tolerance", velocity_tolerance_);
  getInput("threshold_time", threshold_time_);
  getInput("position_buffer_dimension", position_buffer_dimension_);
  position_buffer_.reserve(position_buffer_dimension_);
  publisher_ = node_->create_publisher<std_msgs::msg::Float64>("velocita", 10);

}

void IsMoving::add_position(
  const geometry_msgs::msg::TransformStamped & new_position)
{
  RCLCPP_INFO(node_->get_logger(), "Adding position to buffer dim: %d", position_buffer_.size());
  if(position_buffer_.size() == position_buffer_dimension_){
    buffer_dim_riched_ = true;
  }
  if (buffer_dim_riched_) {
    std::rotate(position_buffer_.rbegin(), position_buffer_.rbegin() + 1, position_buffer_.rend());
    position_buffer_.back() = new_position;
  }
  else
  {
    position_buffer_.push_back(new_position);
  }
}

double IsMoving::compute_velocity()
{
  double total_velocity = 0.0;
  bool at_least_one_couple = false;

  for (size_t i = 1; i < position_buffer_.size(); i++) {
    double delta_x = position_buffer_[i].transform.translation.x - position_buffer_[i - 1].transform.translation.x;
    double delta_y = position_buffer_[i].transform.translation.y - position_buffer_[i - 1].transform.translation.y;
    double time_2 = position_buffer_[i].header.stamp.sec + position_buffer_[i].header.stamp.nanosec* 1e-9;

    double time_1 = position_buffer_[i - 1].header.stamp.sec + position_buffer_[i - 1].header.stamp.nanosec* 1e-9;

    RCLCPP_INFO(node_->get_logger(), "Time 1: %.10f s", time_1);
    RCLCPP_INFO(node_->get_logger(), "Delta time sec: %d s", position_buffer_[i-1].header.stamp.sec);
    RCLCPP_INFO(node_->get_logger(), "Delta time nanosec: %d s", position_buffer_[i-1].header.stamp.nanosec);
    RCLCPP_INFO(node_->get_logger(), "Time 2: %.10f s", time_2);
    RCLCPP_INFO(node_->get_logger(), "Delta time sec 2 : %d s", position_buffer_[i].header.stamp.sec);
    RCLCPP_INFO(node_->get_logger(), "Delta time nanosec 2: %d s", position_buffer_[i].header.stamp.nanosec);

    double delta_time = time_2 - time_1;
    if(delta_time <= 1e-10){
      continue;
    }
    at_least_one_couple = true;
    RCLCPP_INFO(node_->get_logger(), "Delta time: %.10f s", delta_time);
    double velocity = std::sqrt(delta_x * delta_x + delta_y * delta_y ) / delta_time;
    RCLCPP_INFO(node_->get_logger(), "velocity: %f s", velocity);
    RCLCPP_INFO(node_->get_logger(), "Delta pos: %f m", delta_x);

    total_velocity += velocity;
  }
  if (!at_least_one_couple) {
    throw std::runtime_error("No valid couples of data to compute velocity");
  }
  return total_velocity / (position_buffer_.size() - 1);
}


BT::NodeStatus
IsMoving::tick()
{
  RCLCPP_INFO(node_->get_logger(), "IsMoving ticked");

  geometry_msgs::msg::TransformStamped entity_transform_now_msg;
  rclcpp::Time when = node_->get_clock()->now();

  try {
    entity_transform_now_msg = tf_buffer_->lookupTransform(
      frame_,
      "map",
      tf2::TimePointZero);
          // Ottenere la posizione x, y, z dalla trasformazione
    double x = entity_transform_now_msg.transform.translation.x;
    double y = entity_transform_now_msg.transform.translation.y;
    double z = entity_transform_now_msg.transform.translation.z;

    // Ottenere il timestamp dalla trasformazione
    rclcpp::Time timestamp = entity_transform_now_msg.header.stamp;
 
    // if (timestamp.seconds() == 0) {
    //     // Se il timestamp è zero, utilizza il timestamp corrente
    //     timestamp = node_->get_clock()->now();
    // }

    // Stampa posizione e timestamp
    RCLCPP_INFO(
        node_->get_logger(), "Position (x, y, z): %.2f, %.2f, %.2f, Timestamp: %.10f",
        x, y, z, timestamp.nanoseconds()*1e-9);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform %s to %s: %s",
      frame_.c_str(), "map", ex.what());
      RCLCPP_INFO(node_->get_logger(), "Cannot transform");

      return BT::NodeStatus::SUCCESS;
  }
  if(position_buffer_.empty())
  {
    add_position(entity_transform_now_msg);
  }
  else if(std::abs(entity_transform_now_msg.header.stamp.sec + 
                    entity_transform_now_msg.header.stamp.nanosec * 1e-9 - 
                    position_buffer_.back().header.stamp.sec -
                    position_buffer_.back().header.stamp.nanosec * 1-9) > 1e-10)
  {
    add_position(entity_transform_now_msg);
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Skipping position update: timestamp too close to previous");
  }

  if(position_buffer_.size() < position_buffer_dimension_){
          RCLCPP_INFO(node_->get_logger(), "Buffer not full");

    return BT::NodeStatus::SUCCESS;
  }
  double velocity = compute_velocity();
  std_msgs::msg::Float64 dato;
  dato.data = static_cast<double>(velocity);
  publisher_->publish(dato);
  if (velocity >= velocity_tolerance_){
    entity_stopped_ = false;
        RCLCPP_INFO(node_->get_logger(), "Entity is moving at %f m/s", velocity);

    return BT::NodeStatus::SUCCESS;
  }
  RCLCPP_INFO(node_->get_logger(), "Entity is not moving at %f m/s", velocity);
  if(!entity_stopped_)
  {
    first_time_stopped_ = position_buffer_.back().header.stamp;
    entity_stopped_ = true;
    RCLCPP_INFO(node_->get_logger(), "First time stopped %f", velocity);

  }
  RCLCPP_INFO(node_->get_logger(), "Here");

  if((when.seconds() - first_time_stopped_.seconds()) > threshold_time_){

    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace perception


BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<perception::IsMoving>("IsMoving");
}
