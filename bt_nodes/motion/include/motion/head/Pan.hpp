#ifndef HEAD__PAN_HPP_
#define HEAD__PAN_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "motion/head/BTActionNode.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace head
{

class Pan
  : public head::BtActionNode<
    control_msgs::action::FollowJointTrajectory,
    rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit Pan(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({
      BT::InputPort<double>("range"),        // degrees
      BT::InputPort<double>("period"),       // seconds
      BT::InputPort<double>("pitch_angle")   // degrees
    });
  }

private:
  double joint_range_{0.0};
  double period_{0.0};
  double pitch_angle_{0.0};

  const double yaw_limit_{1.3};   // TIAGo limits
  const double pitch_limit_{0.92};
  std::vector<double> yaw_steps_{ 0.0, -0.3, -0.6, 0.3, 0.6 };
  size_t current_step_{0};



  void build_trajectory(
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> & points);
};

}  // namespace head

#endif  // HEAD__PAN_HPP_
