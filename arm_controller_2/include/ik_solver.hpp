#ifndef IK_SOLVER_HPP
#define IK_SOLVER_HPP

#include <rclcpp/rclcpp.hpp>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <std_msgs/msg/string.hpp>
// #include <std_msgs/msg/float64_multi_array.hpp>

#include <memory>
#include <vector>
// #include <Eigen/Dense>

class IkSolver
{
public:
  IkSolver(rclcpp::Node* node);
  std::vector<double> calculateAngles(double x, double y, double z);

private:
  // Callbacks
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);
  // void cartesianCoordinatesCallback(const std_msgs::msg::Float64MultiArray& msg);

  void getJointAngles(
    const double x, const double y, const double z,
    double &shoulder_pan_angle, double &shoulder_lift_angle,
    double &elbow_angle, double &wrist_angle);

  rclcpp::Node* node_;

  // KDL structures
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  // rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cartesian_sub_;
  // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_angles_pub_;
};

#endif // IK_SOLVER_HPP
