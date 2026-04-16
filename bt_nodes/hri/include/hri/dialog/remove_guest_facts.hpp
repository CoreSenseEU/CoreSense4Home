#ifndef HRI__REMOVE_GUEST_FACTS_HPP_
#define HRI__REMOVE_GUEST_FACTS_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace dialog
{

class RemoveGuestFacts : public BT::ActionNodeBase
{
public:
  explicit RemoveGuestFacts(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() override;
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("guest_id")
    };
  }

private:
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr kb_publisher_;
  std::string guest_id_;
};

}  // namespace dialog

#endif  // HRI__REMOVE_GUEST_FACTS_HPP_
