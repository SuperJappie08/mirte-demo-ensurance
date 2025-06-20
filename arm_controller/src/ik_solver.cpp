#include "ik_solver.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl/frames.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <system_error>

IkSolver::IkSolver() : Node("ik_solver")
{
  // Get the package share directory for "arm_controller" using ament_index_cpp
  std::string urdf_file = ament_index_cpp::get_package_share_directory("arm_controller") + "/config/arm.xacro";
  std::string urdf_output_file = "/tmp/mirte_master.urdf";

  // Use system call to convert .xacro to .urdf
  std::string command = "ros2 run xacro xacro " + urdf_file + " > " + urdf_output_file;
  if (std::system(command.c_str()) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to run xacro command to generate URDF");
    return;
  }

  // Load the generated URDF from file
  if (!kdl_parser::treeFromFile(urdf_output_file, tree_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF file: %s", urdf_output_file.c_str());
    return;
  }

  // Get kinematic chain of the arm
  tree_.getChain("base_link", "gripper_center", chain_);
  
  // Create IK solver
  solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_, 1E-15, 500, 1E-15);
}

void IkSolver::getJointAngles(const double x, const double y, const double z,
                               double &shoulder_pan_angle, double &shoulder_lift_angle,
                               double &elbow_angle, double &wrist_angle)
{
  KDL::JntArray q_init(chain_.getNrOfJoints());
  q_init(0) = 0.0;
  q_init(1) = -0.1;
  q_init(2) = -0.1;
  q_init(3) = 0.1;

  KDL::Vector position(x, y, z);
  KDL::Frame end_effector_pose(position);

  KDL::JntArray q_out(chain_.getNrOfJoints());
  
  solver_->CartToJnt(q_init, end_effector_pose, q_out);

  shoulder_pan_angle = q_out(0);
  shoulder_lift_angle = q_out(1);
  elbow_angle = q_out(2);
  wrist_angle = q_out(3);
}

std::vector<double> IkSolver::calculateAngles(double x, double y, double z)
{
  double shoulder_pan_angle, shoulder_lift_angle, elbow_angle, wrist_angle;
  getJointAngles(x, y, z, shoulder_pan_angle, shoulder_lift_angle, elbow_angle, wrist_angle);

  // Create a vector to store the joint angles and return it
  std::vector<double> joint_angles = {shoulder_pan_angle, shoulder_lift_angle, elbow_angle, wrist_angle};
  
  return joint_angles;
}
