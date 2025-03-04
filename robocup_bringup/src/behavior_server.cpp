#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/dummy_behavior.hpp>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ExecuteBTServer : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  using DummyBehavior = nav2_msgs::action::DummyBehavior;
  using GoalHandleDummyBehavior = rclcpp_action::ServerGoalHandle<DummyBehavior>;

  explicit ExecuteBTServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : CascadeLifecycleNode("behavior_server", options)
  {
    this->declare_parameter("plugins", std::vector<std::string>());
    this->declare_parameter("rate", 1.0);
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "Configuring");

    this->get_parameter("plugins", this->plugins_);
    double rate;
    this->get_parameter("rate", rate);
    rate_ = std::make_unique<rclcpp::Rate>(rate);

    for (const auto & plugin : plugins_) {
      RCLCPP_INFO(this->get_logger(), "Loading BT Node: [%s]", plugin.c_str());
      factory_.registerFromPlugin(loader_.getOSName(plugin));
    }
    
    this->action_server_ = rclcpp_action::create_server<DummyBehavior>(
      this,
      "execute_bt",
      std::bind(&ExecuteBTServer::handle_goal, this, _1, _2),
      std::bind(&ExecuteBTServer::handle_cancel, this, _1),
      std::bind(&ExecuteBTServer::handle_accepted, this, _1));
    // Add your configuration code here
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "Activating");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  rclcpp_action::Server<DummyBehavior>::SharedPtr action_server_;

  std::vector<std::string> plugins_;
  std::string bt_xml_file_;
  std::unique_ptr<rclcpp::Rate> rate_;
  BT::BehaviorTreeFactory factory_;
  BT::SharedLibrary loader_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DummyBehavior::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleDummyBehavior> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDummyBehavior> goal_handle)
  {
    std::thread{std::bind(&ExecuteBTServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleDummyBehavior> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<DummyBehavior::Result>();
    RCLCPP_INFO(this->get_logger(), "Loading BT: [%s]", goal->command.data.c_str());
    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", std::static_pointer_cast<rclcpp_cascade_lifecycle::CascadeLifecycleNode>(this->shared_from_this()));
    std::string test_tree = R"(
      <?xml version="1.0"?>
      <root main_tree_to_execute="BehaviorTree">
          <BehaviorTree ID="BehaviorTree">
              <Repeat num_cycles="4">
                  <Delay delay_msec="1000">
                      <Action ID="Rotate" angle="1.57" speed="0.3" />
                  </Delay>
              </Repeat>
          </BehaviorTree>
          <TreeNodesModel>
              <Action ID="Rotate">
                  <input_port default="0.0" name="angle"/>
                  <input_port default="0.0" name="speed"/>
              </Action>
          </TreeNodesModel>
      </root>
    )";  
    BT::Tree tree = factory_.createTreeFromText(test_tree, blackboard);
    while (rclcpp::ok()) {
      if (tree.tickRoot() != BT::NodeStatus::RUNNING) {
        break;
      }
      rate_->sleep();
    }
    goal_handle->succeed(result);

  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<ExecuteBTServer>(node_options);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}