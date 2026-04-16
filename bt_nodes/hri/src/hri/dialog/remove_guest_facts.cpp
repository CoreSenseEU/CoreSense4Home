#include "hri/dialog/remove_guest_facts.hpp"

namespace dialog
{

RemoveGuestFacts::RemoveGuestFacts(
  const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  kb_publisher_ = node_->create_publisher<std_msgs::msg::String>("/kb/remove_fact", 10);
}

BT::NodeStatus RemoveGuestFacts::tick()
{
  RCLCPP_DEBUG(node_->get_logger(), "[RemoveGuestFacts] ticked");

  getInput("guest_id", guest_id_);

  if (!guest_id_.empty()) {
    std_msgs::msg::String fact_msg;
    
    // Remove all facts where the guest is the subject
    fact_msg.data = guest_id_ + " ?p ?o";
    kb_publisher_->publish(fact_msg);

    // Remove any facts where the guest might be the object
    fact_msg.data = "?s ?p " + guest_id_;
    kb_publisher_->publish(fact_msg);

    RCLCPP_INFO(
      node_->get_logger(), "[RemoveGuestFacts] Removing all facts related to: %s",
      guest_id_.c_str());
  } else {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

void RemoveGuestFacts::halt() {}

}  // namespace dialog

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<dialog::RemoveGuestFacts>("RemoveGuestFacts");
}
