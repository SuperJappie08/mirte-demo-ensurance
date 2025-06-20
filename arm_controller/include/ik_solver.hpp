#ifndef IK_SOLVER_HPP
#define IK_SOLVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <vector>

class IkSolver : public rclcpp::Node
{
public:
  IkSolver();
  std::vector<double> calculateAngles(double x, double y, double z);

private:
  void getJointAngles(
    const double x, const double y, const double z,
    double &shoulder_pan_angle, double &shoulder_lift_angle,
    double &elbow_angle, double &wrist_angle);

  KDL::Tree tree_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_;
};

#endif // IK_SOLVER_HPP
