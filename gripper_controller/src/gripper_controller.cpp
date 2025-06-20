#include <memory>
#include <string>
#include <chrono>
#include <cmath>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "mirte_msgs/msg/servo_position.hpp"

using GripperCommand = control_msgs::action::GripperCommand;
using SetBool = std_srvs::srv::SetBool;
using ServoPosition = mirte_msgs::msg::ServoPosition;
using namespace std::chrono_literals;

class GripperControllerNode : public rclcpp::Node
{
public:
  GripperControllerNode()
  : Node("gripper_controller_node")
  {
    gripper_action_client_ = rclcpp_action::create_client<GripperCommand>(
      this, "/mirte_master_gripper_controller/gripper_cmd");

    while (!gripper_action_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(), "Waiting for gripper action server...");
    }

    gripper_service_ = this->create_service<SetBool>(
      "toggle_gripper",
      std::bind(&GripperControllerNode::handle_gripper_request, this,
                std::placeholders::_1, std::placeholders::_2));

    gripper_position_sub_ = this->create_subscription<ServoPosition>(
      "/io/servo/hiwonder/gripper/position", 10,
      std::bind(&GripperControllerNode::gripper_position_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Gripper controller node ready.");
  }

private:
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
  rclcpp::Service<SetBool>::SharedPtr gripper_service_;
  rclcpp::Subscription<ServoPosition>::SharedPtr gripper_position_sub_;
  std::atomic<double> latest_gripper_position_{0.0};

  rclcpp::TimerBase::SharedPtr monitor_timer_;
  double target_position_;
  rclcpp::Time goal_start_time_;
  rclcpp::Duration timeout_ = rclcpp::Duration::from_seconds(10.0);

  void gripper_position_callback(const ServoPosition::SharedPtr msg)
  {
    latest_gripper_position_.store(msg->angle);
  }

  void handle_gripper_request(
    const std::shared_ptr<SetBool::Request> request,
    std::shared_ptr<SetBool::Response> response)
  {
    double open = 0.0;
    double close = -0.3;
    target_position_ = request->data ? open : close;

    // Send the command
    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = target_position_;
    RCLCPP_INFO(this->get_logger(), "Sending gripper command with position: %.2f", target_position_);
    gripper_action_client_->async_send_goal(goal_msg);

    // Respond immediately (SetBool requires sync response)
    response->success = true;
    response->message = "Gripper command sent. Monitoring in background.";

    // Start background monitor timer
    goal_start_time_ = this->now();
    monitor_timer_ = this->create_wall_timer(
      100ms,
      std::bind(&GripperControllerNode::check_gripper_progress, this));
  }

  void check_gripper_progress()
  {
    double actual_pos = latest_gripper_position_.load();
    double error = std::abs(actual_pos - target_position_);

    RCLCPP_INFO(this->get_logger(), "Checking gripper position. Actual: %.2f, Target: %.2f", actual_pos, target_position_);

    if (error <= 0.05) {
      monitor_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "Gripper reached target position.");
    } else if ((this->now() - goal_start_time_) > timeout_) {
      monitor_timer_->cancel();
      RCLCPP_WARN(this->get_logger(), "Gripper did not reach target position in time.");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
