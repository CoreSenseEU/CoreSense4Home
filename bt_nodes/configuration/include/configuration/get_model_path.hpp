#ifndef CONFIGURATION__GET_MODEL_PATH_HPP_
#define CONFIGURATION__GET_MODEL_PATH_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

namespace configuration
{

class GetModelPath : public BT::SyncActionNode
{
public:
  explicit GetModelPath(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({
      BT::InputPort<std::string>("model"),
      BT::OutputPort<std::string>("model_path")
    });
  }
};

}  // namespace configuration

#endif  // CONFIGURATION__GET_MODEL_PATH_HPP_
