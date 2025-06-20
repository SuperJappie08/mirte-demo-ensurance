#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"



using NavigateToPose = nav2_msgs::action::NavigateToPose;
using Point = geometry_msgs::msg::Point;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;


namespace BT
{

template<>
[[nodiscard]] Pose convertFromString<Pose>(StringView str);

template<>
[[nodiscard]] std::string toStr<Pose>(const Pose & direction);

template<>
Pose convertFromString<Pose>(StringView str)
{
  std::vector<double> pose_parts = convertFromString<std::vector<double>>(str);
  if (pose_parts.size() != 7) {
    throw RuntimeError(
            std::string("Cannot convert this to Pose: ") +
            static_cast<std::string>(str));
  }
  Pose pose;
  pose.position.x = pose_parts[0];
  pose.position.y = pose_parts[1];
  pose.position.z = pose_parts[2];
  pose.orientation.x = pose_parts[3];
  pose.orientation.y = pose_parts[4];
  pose.orientation.z = pose_parts[5];
  pose.orientation.w = pose_parts[6];
  return pose;
}

template<>
std::string toStr<Pose>(const Pose & pose)
{
  std::stringstream ss;
  ss << pose.position.x << ";" << pose.position.y << ";" << pose.position.z << ";"
     << pose.orientation.x << ";" << pose.orientation.y << ";" << pose.orientation.z
     << ";" << pose.orientation.w;
  return ss.str();
}

std::ostream & operator<<(std::ostream & os, const Pose & pose)
{
  os << toStr(pose);
  return os;
}


class FineNavigationAction : public RosActionNode<NavigateToPose>
{
public:
  //Name for the pose input port
  static constexpr const char * POSE = "in_tree_pose";
  static constexpr const char * TARGET = "in_target";


  FineNavigationAction(
    const std::string & name,
    const NodeConfig & conf,
    const RosNodeParams & params)
  : RosActionNode<NavigateToPose>(name, conf, params)
  {
    std::cout << "Someone made me (an FineNavigationAction Action Nodee)" << std::endl;

    // RCLCPP_INFO(logger(), node_->get_name());

  }

  static PortsList providedPorts()
  {
    PortsList base_ports = RosActionNode::providedPorts();
    PortsList child_ports = {
      InputPort<Pose>(POSE),
      InputPort<Point>(TARGET)
    };
    child_ports.merge(base_ports);
    return child_ports;
  }

  bool setGoal(RosActionNode::Goal & goal) override
  {
    // #goal definition
    // geometry_msgs/PoseStamped pose
    // string behavior_tree
    std::stringstream ss;

    ss << "setGoal in pose";

    getInput(POSE, tree_pose);
    getInput(TARGET, target_point);

    const double offset = 0.2;

    // Tree's original orientation
    tf2::Quaternion q_original(tree_pose.orientation.x,
        tree_pose.orientation.y,
        tree_pose.orientation.z,
        tree_pose.orientation.w);

    // 180° rotation quaternion
    tf2::Quaternion q_180;
    q_180.setRPY(0, 0, M_PI);

    // Calculate rotated quaternion (used only for offset)
    tf2::Quaternion q_rotated = q_original; // q_180 * q_original;
    // q_rotated.normalize();

    // Extract yaw from rotated quaternion (used for offset only)
    double roll, pitch, rotated_yaw;
    tf2::Matrix3x3(q_rotated).getRPY(roll, pitch, rotated_yaw);

    // Use rotated yaw to calculate offset position
    double goal_x = target_point.x - offset * std::cos(rotated_yaw);
    double goal_y = target_point.y - offset * std::sin(rotated_yaw);
    Pose pose_to_navigate_to = Pose();
    pose_to_navigate_to.position.x = goal_x;
    pose_to_navigate_to.position.y = goal_y;
    pose_to_navigate_to.position.z = 0.0;

    tf2::Quaternion q(0, 0, q_rotated.z(), q_rotated.w());
    pose_to_navigate_to.orientation = tf2::toMsg(q);

    PoseStamped stamped_pose;
    stamped_pose.header.frame_id = "map";
    stamped_pose.header.stamp = now();
    stamped_pose.pose = pose_to_navigate_to;

    goal.pose = stamped_pose;

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
    ss << "NavigateToPose Result received";
    //Unfortunately, NavigateToPose in Nav2 right now provides no actual result indicating you reached the pose or not..

    RCLCPP_INFO(logger(), ss.str().c_str());

    RCLCPP_INFO(logger(), "SUCCESS IN RESULT RCV GOTOPOSE");

    return NodeStatus::SUCCESS;
  }

  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_INFO(logger(), "Here we are");
    RCLCPP_ERROR(logger(), "Navigate to Pose Error: %d", error);
    return NodeStatus::FAILURE;
  }

  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    return NodeStatus::RUNNING;
  }

private:
  Pose tree_pose;
  Point target_point;
};

BT_REGISTER_ROS_NODES(factory, params)
{
  RosNodeParams aug_params;
  aug_params.nh = params.nh;
  aug_params.server_timeout = std::chrono::milliseconds(40000);   //Nav2 can take a while to respond, especialy in a container.
  //TODO: options.use_global_arguments(false) need to fix this for plguins somehow. also applies to client name
  factory.registerNodeType<FineNavigationAction>("FineNavigation", aug_params);
}

}