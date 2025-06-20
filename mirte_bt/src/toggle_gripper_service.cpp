#include <iostream>
#include <vector>
#include <algorithm>
#include "std_srvs/srv/set_bool.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "control_msgs/action/gripper_command.hpp"


namespace BT
{

using GripperCommand = control_msgs::action::GripperCommand;


class GripperCommanddAction : public RosActionNode<control_msgs::action::GripperCommand>
{
public:
  //Name for the pose input port
  static constexpr const char * OPEN_CLOSE = "in_open";

  GripperCommanddAction(
    const std::string & name,
    const NodeConfig & conf,
    const RosNodeParams & params)
  : RosActionNode<control_msgs::action::GripperCommand>(name, conf, params)
  {
    std::cout << "Someone made me (an GripperCommandAction Action Nodee)" << std::endl;

    // RCLCPP_INFO(logger(), node_->get_name());

  }

  static PortsList providedPorts()
  {
    PortsList base_ports = RosActionNode::providedPorts();
    PortsList child_ports = {
      InputPort<bool>(OPEN_CLOSE),
    };
    child_ports.merge(base_ports);
    return child_ports;
  }

  bool setGoal(RosActionNode::Goal & goal) override
  {

    getInput(OPEN_CLOSE, open_close);
    double open = 0.0;
    double close = -0.3;
    target_position_ = open_close ? open : close;

    goal.command.position = target_position_;
   
    return true;
  }

  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    // #result definition
    // std_msgs/Empty result
    std::stringstream ss;
    ss << "GripperCommand Result received";
    //Unfortunately, GripperCommand in Nav2 right now provides no actual result indicating you reached the pose or not..

    RCLCPP_INFO(logger(), ss.str().c_str());

    RCLCPP_INFO(logger(), "SUCCESS IN RESULT RCV GRIPPERCOMMAND");

    return NodeStatus::SUCCESS;
  }

  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_INFO(logger(), "Here we are");
    RCLCPP_ERROR(logger(), "GRIPER COMMAND Error: %d", error);
    return NodeStatus::FAILURE;
  }

  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    return NodeStatus::RUNNING;
  }

private:
  bool open_close;
  double target_position_;
};

BT_REGISTER_ROS_NODES(factory, params)
{
  RosNodeParams aug_params;
  aug_params.nh = params.nh;
  aug_params.server_timeout = std::chrono::milliseconds(40000);   //Nav2 can take a while to respond, especialy in a container.
  //TODO: options.use_global_arguments(false) need to fix this for plguins somehow. also applies to client name
  factory.registerNodeType<GripperCommanddAction>("GripperCommand", aug_params);
}

}