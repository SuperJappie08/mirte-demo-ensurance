#include "ik_solver.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Dense>

IkSolver::IkSolver(rclcpp::Node*node) : node_(node)
{
  // Subscribe to the robot_description topic instead of loading from file
  robot_description_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/robot_description",
    rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
    std::bind(&IkSolver::robotDescriptionCallback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(node_->get_logger(), "Waiting for robot_description...");

  // // Subscribe to Cartesian coordinates
  // cartesian_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
  //   "goal_arm_cartesian_coordinates", 
  //   rclcpp::QoS(rclcpp::KeepLast(1)),
  //   std::bind(&IkSolver::cartesianCoordinatesCallback, this, std::placeholders::_1));

  // joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
  //   "/joint_states", 10,
  //   std::bind(&MoveArmServer::joint_state_callback, this, std::placeholders::_1));

  // Publisher for joint angles in radians
  // joint_angles_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("goal_arm_joint_states", rclcpp::QoS(10));
}

// void IkSolver::cartesianCoordinatesCallback(const std_msgs::msg::Float64MultiArray& msg)
// {
//   double x = msg.data[0];
//   double y = msg.data[1];
//   double z = msg.data[2];

//   calculateAngles(x, y, z);
// }

void IkSolver::getJointAngles(
  const double x, const double y, const double z, 
  double & shoulder_pan_angle, double & shoulder_lift_angle, 
  double & elbow_angle, double & wrist_angle)
{
  RCLCPP_WARN(this->node_->get_logger(), "KAAS4 %d", chain_.getNrOfJoints());

  KDL::JntArray q_init(chain_.getNrOfJoints());
  q_init(0) = 0.1;
  q_init(1) = 0.2;
  q_init(2) = -0.3;
  q_init(3) = 0.1;
  RCLCPP_WARN(this->node_->get_logger(), "KAAS5");


  KDL::Vector position(x, y, z);
  KDL::Rotation rotation = KDL::Rotation::RPY(0.0, 0.0, 0.0);
  KDL::Frame end_effector_pose(rotation, position);
  //KDL::Frame pos(position);

  KDL::JntArray q_out(chain_.getNrOfJoints());
  
  RCLCPP_INFO(node_->get_logger(), "%s", solver_->strError(solver_->CartToJnt(q_init, end_effector_pose, q_out)));

  shoulder_pan_angle = q_out(0);
  shoulder_lift_angle = q_out(1);
  elbow_angle = q_out(2);
  wrist_angle = q_out(3);
}

std::vector<double> IkSolver::calculateAngles(double x, double y, double z)
{
  double shoulder_pan_angle, shoulder_lift_angle, elbow_angle, wrist_angle;
  RCLCPP_WARN(this->node_->get_logger(), "KAAS3");
  getJointAngles(x, y, z, shoulder_pan_angle, shoulder_lift_angle, elbow_angle, wrist_angle);
  
  RCLCPP_INFO(this->node_->get_logger(), "SOLVED: shoulder pan, lift, elbow and wrist joint angles: %.2f, %.2f, %.2f, %.2f radians",
              shoulder_pan_angle, shoulder_lift_angle, elbow_angle, wrist_angle);

  return {shoulder_pan_angle, shoulder_lift_angle, elbow_angle, wrist_angle};
}

void IkSolver::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(msg->data, tree)) {
    RCLCPP_ERROR(this->node_->get_logger(), "Failed to parse URDF from robot_description.");
    return;
  }


  if(!tree.getSubTree(
    "shoulder_link", tree_
  )){
    RCLCPP_ERROR(this->node_->get_logger(), "Failed to get subtree for shoulder_link.");
    return;
  }

  // tree_ = tree2;
    RCLCPP_ERROR(this->node_->get_logger(), "%s", tree_.getRootSegment()->first.c_str());


  RCLCPP_INFO(this->node_->get_logger(), "nb joints:        %d", tree_.getNrOfJoints());
  RCLCPP_INFO(this->node_->get_logger(), "nb segments:      %d", tree_.getNrOfSegments());
  RCLCPP_INFO(this->node_->get_logger(), "root segment:     %s", tree_.getRootSegment()->first.c_str());

  if (!tree_.getChain("shoulder_link", "gripper_center", chain_)) {
    RCLCPP_ERROR(this->node_->get_logger(), "Failed to extract chain from base_link to gripper_center");
    return;
  }

  RCLCPP_INFO(this->node_->get_logger(), "chain nb joints:  %d", chain_.getNrOfJoints());
Eigen::Matrix<double, 6,1> l;
l << 1., 1., 1., 0.01, 0.3, 0.01;
  solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_, l, 1E-5, 5000);
  // solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_, 1E-5, 5000);

}

