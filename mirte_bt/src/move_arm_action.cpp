#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "fsm/action/move_arm.hpp"
#include "behaviortree_ros2/plugins.hpp"

using MoveArm = fsm::action::MoveArm;
using Point = geometry_msgs::msg::Point;

namespace BT
{
class MoveArmAction : public RosActionNode<MoveArm>
{
public:
  //Name for the pose input port
  static constexpr const char * HOME = "in_home";
  static constexpr const char * TARGET = "in_target";


  MoveArmAction(
    const std::string & name,
    const NodeConfig & conf,
    const RosNodeParams & params)
  : RosActionNode<MoveArm>(name, conf, params)
  {
    std::cout << "Someone made me (an MoveArmAction Action Nodee)" << std::endl;

    // RCLCPP_INFO(logger(), node_->get_name());

  }

  static PortsList providedPorts()
  {
    PortsList base_ports = RosActionNode::providedPorts();
    PortsList child_ports = {
      InputPort<Point>(TARGET),
      InputPort<bool>(HOME),
    };
    child_ports.merge(base_ports);
    return child_ports;
  }

  bool setGoal(RosActionNode::Goal & goal) override
  {
    // #goal definition
    // geometry_msgs/PoseStamped pose
    // string behavior_tree


    getInput(HOME, home);
    getInput(TARGET, target_point);

    std::stringstream ss;

    ss << "setGoal in MoveArm ";
    ss << " x = " << target_point.x;
    ss << " y = " << target_point.y;
    ss << " z = " << target_point.z;
    ss << " home = " << home;

    goal.x = target_point.x;
    goal.y = target_point.y;
    goal.z = target_point.z;
    goal.home = home;


    RCLCPP_INFO(logger(), ss.str().c_str());

    return true;
  }

  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    // #result definition
    // std_msgs/Empty result
    std::stringstream ss;
    ss << "MoveArm Result received";
   
    RCLCPP_INFO(logger(), ss.str().c_str());

    RCLCPP_INFO(logger(), "SUCCESS IN RESULT RCV MoveArm");

    return NodeStatus::SUCCESS;
  }

  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(logger(), "MoveArm Error: %d", error);
    return NodeStatus::FAILURE;
  }

  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    std::stringstream ss;
    ss << "MoveArm Feedback received";
    ss << feedback.get()->status;
    RCLCPP_INFO(logger(), ss.str().c_str());
    return NodeStatus::RUNNING;
  }

private:
  bool home;
  Point target_point;
};

BT_REGISTER_ROS_NODES(factory, params)
{
  RosNodeParams aug_params;
  aug_params.nh = params.nh;
  aug_params.server_timeout = std::chrono::milliseconds(40000);
  //TODO: options.use_global_arguments(false) need to fix this for plguins somehow. also applies to client name
  factory.registerNodeType<MoveArmAction>("MoveArm", aug_params);
}

}