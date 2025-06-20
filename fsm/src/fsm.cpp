#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "fsm/action/move_arm.hpp"
#include "fsm/srv/get_point.hpp"
#include "fsm/srv/send_state.hpp"

using namespace std::chrono_literals;

struct TreePose
{
  std::string name;
  double x;
  double y;
  double qz;
  double qw;
};

class FSM : public rclcpp::Node
{
public:
  //Navigation
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  //Arm Manipulation
  using MoveArm = fsm::action::MoveArm;
  using ArmGoalHandle = rclcpp_action::ClientGoalHandle<MoveArm>;

  enum class State
  {
    // CONNECTING,
    REST,
    NAVIGATING,
    SCAN_TREE,
    FINE_NAVIGATION,
    ARM_MANIPULATION_TREE,
    GRASP_APPLE,
    HOME_ARM_1,
    GOTO_BASKET,
    ARM_MANIPULATION_BASKET,
    RELEASE_APPLE,
    HOME_ARM_2
  };

  FSM()
  : Node("fsm_node"), state_(State::REST)
  {
    std::vector<double> default_vec;

    //tree poses
    this->declare_parameter("tree_pose1", rclcpp::ParameterValue(default_vec));
    this->declare_parameter("tree_pose2", rclcpp::ParameterValue(default_vec));
    this->declare_parameter("tree_pose3", rclcpp::ParameterValue(default_vec));
    //basket location
    this->declare_parameter("basket", rclcpp::ParameterValue(default_vec));

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose"); //Client for base navigation
    arm_client_ = rclcpp_action::create_client<MoveArm>(this, "move_arm");            //Client for arm manipulation
    point_client_ = this->create_client<fsm::srv::GetPoint>("get_point"); //Client to request detection from Harm
    gripper_client_ = this->create_client<std_srvs::srv::SetBool>("toggle_gripper"); //Client to open or close gripper.

    timer_ = this->create_wall_timer(1s, std::bind(&FSM::loop, this));

    debug_pub_ = this->create_publisher<std_msgs::msg::String>("fsm_debug", 10);

    send_state_service_ = this->create_service<fsm::srv::SendState>(
      "send_state",
      std::bind(&FSM::handle_send_state, this, std::placeholders::_1, std::placeholders::_2));

    load_goals();

  }

private:
  State state_;
  // int retry_count_ = 0;
  // const int max_retries_ = 10;
  size_t current_tree_index_ = 0;
  std::vector<TreePose> tree_poses_;
  double basket_x_ = 0.0;
  double basket_y_ = 0.0;
  double basket_qz_ = 0.0;
  double basket_qw_ = 0.0;

  bool waiting_on_nav_ = false;
  bool waiting_on_arm_ = false;
  bool waiting_for_point_ = false;
  bool waiting_on_gripper_ = false;

  rclcpp_action::Client<MoveArm>::SharedPtr arm_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::Client<fsm::srv::GetPoint>::SharedPtr point_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gripper_client_;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point3d_sub_;

  rclcpp::Service<fsm::srv::SendState>::SharedPtr send_state_service_;

  geometry_msgs::msg::Point target_apple_point_;

  void loop()
  {
    switch (state_) {
      // case State::CONNECTING: {
      //   publish_debug("State: CONNECTING");

      //   // Check both action servers
      //   bool nav_ready = client_->wait_for_action_server(2s);
      //   bool arm_ready = arm_client_->wait_for_action_server(2s);

      //   if (nav_ready && arm_ready) {
      //     publish_debug("Connected to action servers.");
      //     state_ = State::REST;
      //   } else if (++retry_count_ >= max_retries_) {
      //     publish_debug("Failed to connect to one or more action servers. Moving to REST.");
      //     state_ = State::REST;
      //   } else {
      //     publish_debug("Waiting for action servers...");
      //   }
      //   break;
      // }

      case State::REST: {
          publish_debug("State: REST");
          // 1. Cancel any ongoing navigation goals
          if (client_ && waiting_on_nav_) {
            auto cancel_future = client_->async_cancel_all_goals();
            // Optionally handle cancel result with a callback or wait for it
            waiting_on_nav_ = false;
            publish_debug("Cancelled navigation goals.");
          }

          // 2. Cancel any ongoing arm goals
          if (arm_client_ && waiting_on_arm_) {
            auto cancel_future = arm_client_->async_cancel_all_goals();
            waiting_on_arm_ = false;
            publish_debug("Cancelled arm goals.");
          }
          current_tree_index_ = 0;

          waiting_on_nav_ = false;
          waiting_on_arm_ = false;
          waiting_for_point_ = false;
          waiting_on_gripper_ = false;
          break;
        }

      case State::NAVIGATING: {
          publish_debug("State: NAVIGATING");
          if (!waiting_on_nav_) {
            if (current_tree_index_ < tree_poses_.size()) {
              const auto & tree = tree_poses_[current_tree_index_];
              publish_debug("Navigating to scan " + tree.name);

              const double offset = 0.2;

              // Tree's original orientation
              tf2::Quaternion q_original(0, 0, tree.qz, tree.qw);

              // 180° rotation quaternion
              tf2::Quaternion q_180;
              q_180.setRPY(0, 0, M_PI*0.5);

              // Calculate rotated quaternion (used only for offset)
              tf2::Quaternion q_rotated = q_180 * q_original;
              q_rotated.normalize();

              // Extract yaw from rotated quaternion (used for offset only)
              double roll, pitch, rotated_yaw;
              tf2::Matrix3x3(q_rotated).getRPY(roll, pitch, rotated_yaw);

              // Use rotated yaw to calculate offset position
              double goal_x = tree.x - offset * std::cos(rotated_yaw);
              double goal_y = tree.y - offset * std::sin(rotated_yaw);

              // Send goal with the ORIGINAL orientation (not rotated)
              send_goal(
                goal_x, goal_y, q_rotated.z(), q_rotated.w(), [this]() {
                  state_ = State::SCAN_TREE;
                });

              waiting_on_nav_ = true;
            } else {
              publish_debug("All trees processed. Going to REST.");
              state_ = State::REST;
            }
          }
          break;
        }

      case State::SCAN_TREE: {
          publish_debug("State: SCAN_TREE");

          if (waiting_for_point_) {
            // Waiting for service or point message
            publish_debug("Waiting for point");
            break;
          }

          if (!point_client_->wait_for_service(1s)) {
            publish_debug("Waiting for get_point service...");
            break;
          }

          auto request = std::make_shared<fsm::srv::GetPoint::Request>();
          waiting_for_point_ = true;

          point_client_->async_send_request(
            request,
            [this](rclcpp::Client<fsm::srv::GetPoint>::SharedFuture future) {
              auto response = future.get();

              if (response->success) {
                publish_debug("GetPoint service succeeded, subscribing to /point_3D");

                if (point3d_sub_) {
                  publish_debug("POINT3D SUB EXISTED ALREADY");
                  point3d_sub_.reset();
                }
                else{
                  point3d_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
                    "/point_3D", 10,
                    [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                      publish_debug(
                        "Received 3D point from /point_3D: x=" + std::to_string(msg->x) +
                        ", y=" + std::to_string(msg->y) +
                        ", z=" + std::to_string(msg->z));

                      target_apple_point_ = *msg; // Save the point

                      // Cleanup
                      point3d_sub_.reset();
                      waiting_for_point_ = false;

                      // Move to next state
                      state_ = State::FINE_NAVIGATION;
                    });
                }
              } else {
                publish_debug("GetPoint service failed.");
                waiting_for_point_ = false;
                state_ = State::REST; // or retry
              }
            }
          );

          break;
        }

      case State::FINE_NAVIGATION: {
          publish_debug("State: FINE_NAVIGATION");
          if (!waiting_on_nav_) {
            if (current_tree_index_ < tree_poses_.size()) {
              const auto & tree = tree_poses_[current_tree_index_];
              publish_debug("Fine navigating to apple");

              const double offset = 0.2;

              // Tree's original orientation
              // tf2::Quaternion q_original(0, 0, tree.qz, tree.qw);
              tf2::Quaternion q_original(0, 0, 0, 1);
              q_original.setRPY(0, 0, tree.qw);

              // 180° rotation quaternion
              tf2::Quaternion q_180;
              q_180.setRPY(0, 0, M_PI*0);

              // Calculate rotated quaternion (used only for offset)
              tf2::Quaternion q_rotated = q_180 * q_original;
              q_rotated.normalize();

              // Extract yaw from rotated quaternion (used for offset only)
              double roll, pitch, rotated_yaw;
              tf2::Matrix3x3(q_rotated).getRPY(roll, pitch, rotated_yaw);

              // Use rotated yaw to calculate offset position
              double goal_x = target_apple_point_.x - offset * std::cos(rotated_yaw);
              double goal_y = target_apple_point_.y - offset * std::sin(rotated_yaw);
              // publish_debug("goal_x ----------", goal_x);
              // publish_debug("goal_y ----------", goal_y);
              send_goal(
                goal_x, goal_y, q_rotated.z(), q_rotated.w(), [this]() {
                  state_ = State::ARM_MANIPULATION_TREE;
                });
              waiting_on_nav_ = true;
            } else {
              publish_debug("All trees processed. Going to REST.");
              state_ = State::REST;
            }
          }
          break;
        }

      case State::ARM_MANIPULATION_TREE: {
          publish_debug("State: ARM_MANIPULATION_TREE");
          if (!waiting_on_arm_) {
            send_arm_goal(
              target_apple_point_.x,
              target_apple_point_.y,
              target_apple_point_.z,
              false,
              [this]() {
                state_ = State::GRASP_APPLE;
              });
          }
          break;
        }

      case State::GRASP_APPLE: {
          publish_debug("State: GRASP_APPLE");
          if (!waiting_on_gripper_) {
            waiting_on_gripper_ = true;
            trigger_gripper(
              true, [this]() {
                waiting_on_gripper_ = false;
                state_ = State::HOME_ARM_1;
              });
          }
          break;
        }


      case State::HOME_ARM_1: {
          publish_debug("State: HOME_ARM_1");
          if (!waiting_on_arm_) {
            send_arm_goal(
              target_apple_point_.x,
              target_apple_point_.y,
              target_apple_point_.z,
              true,
              [this]() {
                state_ = State::GOTO_BASKET;
              });
          }
          break;
        }

      case State::GOTO_BASKET: {
          publish_debug("State: GOTO_BASKET");
          if (!waiting_on_nav_) {
            const double offset = 0.3;

            // Tree's original orientation
            tf2::Quaternion q_original(0, 0, basket_qz_, basket_qw_);

            // 180° rotation quaternion
            tf2::Quaternion q_180;
            q_180.setRPY(0, 0, M_PI);

            // Calculate rotated quaternion (used only for offset)
            tf2::Quaternion q_rotated = q_180 * q_original;
            q_rotated.normalize();

            // Extract yaw from rotated quaternion (used for offset only)
            double roll, pitch, rotated_yaw;
            tf2::Matrix3x3(q_rotated).getRPY(roll, pitch, rotated_yaw);

            // Use rotated yaw to calculate offset position
            double goal_x = basket_x_ - offset * std::cos(rotated_yaw);
            double goal_y = basket_y_ - offset * std::sin(rotated_yaw);

            send_goal(
              goal_x, goal_y, q_rotated.z(), q_rotated.w(), [this]() {
                ++current_tree_index_;
                state_ = State::ARM_MANIPULATION_BASKET;
              });
            waiting_on_nav_ = true; // Prevent multiple goal sends
          }
          break;
        }

      case State::ARM_MANIPULATION_BASKET: {
          publish_debug("State: ARM_MANIPULATION_BASKET"); //Modify to give it the basket points.
          if (!waiting_on_arm_) {
            send_arm_goal(
              basket_x_,
              basket_y_,
              0.4,
              false,
              [this]() {
                state_ = State::RELEASE_APPLE;
              });
          }
          break;
        }

      case State::RELEASE_APPLE: {
          publish_debug("State: RELEASE_APPLE");
          if (!waiting_on_gripper_) {
            waiting_on_gripper_ = true;
            trigger_gripper(
              false, [this]() {
                waiting_on_gripper_ = false;
                state_ = State::HOME_ARM_2;
              });
          }
          break;
        }

      case State::HOME_ARM_2: {
          publish_debug("State: HOME_ARM_2");
          if (!waiting_on_arm_) {
            send_arm_goal(
              target_apple_point_.x,
              target_apple_point_.y,
              target_apple_point_.z,
              false,
              [this]() {
                state_ = State::NAVIGATING;
              });
          }
          break;
        }
    }
  }


  void send_goal(double x, double y, double qz, double qw, std::function<void()> on_success)
  {
    if (!client_->wait_for_action_server(2s)) {
      publish_debug("Navigation action server not available.");
      state_ = State::REST;
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();

    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q(0, 0, qz, qw);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.feedback_callback = [this](GoalHandle::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        publish_debug(
          "Feedback: Distance remaining = " +
          std::to_string(feedback->distance_remaining));
      };

    options.result_callback = [this, on_success](const GoalHandle::WrappedResult & result) {
        publish_debug("Goal result code: " + std::to_string(static_cast<int>(result.code)));
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          publish_debug("Goal reached successfully.");
          waiting_on_nav_ = false;
          on_success();
        } else {
          publish_debug("Goal failed. Going to REST.");
          waiting_on_nav_ = false;
          state_ = State::REST;
        }
      };

    client_->async_send_goal(goal_msg, options);
  }

  void publish_debug(const std::string & msg)
  {
    std_msgs::msg::String debug_msg;
    debug_msg.data = "[" + std::to_string(this->now().seconds()) + "] " + msg;
    debug_pub_->publish(debug_msg);
  }

  void send_arm_goal(double x, double y, double z, bool home, std::function<void()> on_success)
  {
    if (!arm_client_->wait_for_action_server(2s)) {
      publish_debug("Arm action server not available.");
      state_ = State::REST;
      return;
    }

    auto goal_msg = MoveArm::Goal();
    goal_msg.x = x;
    goal_msg.y = y;
    goal_msg.z = z;
    goal_msg.home = home;

    publish_debug(
      "Sending arm goal: x=" + std::to_string(x) + ", y=" + std::to_string(y) +
      ", z=" + std::to_string(z) + ", home=" + std::string(home ? "true" : "false"));

    auto options = rclcpp_action::Client<MoveArm>::SendGoalOptions();

    options.feedback_callback = [this](ArmGoalHandle::SharedPtr,
        const std::shared_ptr<const MoveArm::Feedback> feedback) {
        publish_debug("Arm feedback: " + feedback->status);
      };

    options.result_callback = [this, on_success](const ArmGoalHandle::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
          publish_debug("Arm movement succeeded: " + result.result->message);
          on_success();
        } else {
          publish_debug("Arm movement failed: " + result.result->message);
          state_ = State::REST;
        }
        waiting_on_arm_ = false;
      };

    arm_client_->async_send_goal(goal_msg, options);
    waiting_on_arm_ = true;
  }

  void load_goals()
  {
    const std::vector<std::string> tree_names = {"tree_pose1", "tree_pose2", "tree_pose3"};

    for (const auto & name : tree_names) {
      std::vector<double> pose_vec;
      if (this->get_parameter(name, pose_vec)) {
        if (pose_vec.size() == 4) {
          tree_poses_.push_back({name, pose_vec[0], pose_vec[1], pose_vec[2], pose_vec[3]});
          publish_debug(
            "Loaded " + name + ": x=" + std::to_string(pose_vec[0]) +
            ", y=" + std::to_string(pose_vec[1]) +
            ", qz=" + std::to_string(pose_vec[2]) +
            ", qw=" + std::to_string(pose_vec[3]));
        } else {
          publish_debug("Invalid " + name + ": needs exactly 4 elements (x, y, qz, qw).");
        }
      } else {
        publish_debug("Parameter " + name + " not found.");
      }
    }

    std::vector<double> basket_vec;
    if (this->get_parameter("basket", basket_vec)) {
      if (basket_vec.size() == 4) {
        basket_x_ = basket_vec[0];
        basket_y_ = basket_vec[1];
        basket_qz_ = basket_vec[2];
        basket_qw_ = basket_vec[3];
        publish_debug(
          "Loaded basket position: x=" + std::to_string(basket_x_) +
          ", y=" + std::to_string(basket_y_) +
          ", qz=" + std::to_string(basket_qz_) +
          ", qw=" + std::to_string(basket_qw_));
      } else {
        publish_debug("Invalid basket: needs exactly 4 elements (x, y, qz, qw).");
      }
    } else {
      publish_debug("Parameter basket not found.");
    }
  }

  void trigger_gripper(bool close, std::function<void()> on_done)
  {
    if (!gripper_client_->wait_for_service(1s)) {
      publish_debug("Gripper service not available.");
      state_ = State::REST;
      return;
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = close;

    publish_debug(std::string("Sending gripper command: ") + (close ? "CLOSE" : "OPEN"));

    gripper_client_->async_send_request(
      request,
      [this, close, on_done](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
          publish_debug(std::string("Gripper ") + (close ? "closed" : "opened") + " successfully.");
          on_done();
        } else {
          publish_debug("Gripper action failed");
          state_ = State::REST;
        }
      }
    );
  }

  void handle_send_state(
    const std::shared_ptr<fsm::srv::SendState::Request> request,
    std::shared_ptr<fsm::srv::SendState::Response> response)
  {
    std::string input = request->command;
    std::transform(input.begin(), input.end(), input.begin(), ::toupper);

    static const std::unordered_map<std::string, State> string_to_state = {
      {"REST", State::REST},
      {"SCAN_TREE", State::SCAN_TREE},
      {"FINE_NAVIGATION", State::FINE_NAVIGATION},
      {"ARM_MANIPULATION_TREE", State::ARM_MANIPULATION_TREE},
      {"GRASP_APPLE", State::GRASP_APPLE},
      {"HOME_ARM_1", State::HOME_ARM_1},
      {"GOTO_BASKET", State::GOTO_BASKET},
      {"ARM_MANIPULATION_BASKET", State::ARM_MANIPULATION_BASKET},
      {"RELEASE_APPLE", State::RELEASE_APPLE},
      {"HOME_ARM_2", State::HOME_ARM_2},
      {"NAVIGATING", State::NAVIGATING},
    };

    auto it = string_to_state.find(input);
    if (it != string_to_state.end()) {
      state_ = it->second;
      std::string msg = "FSM state manually changed to: " + input;
      publish_debug(msg);
      response->success = true;
    } else {
      std::string msg = "Invalid state requested: " + input;
      publish_debug(msg);
      response->success = false;
    }
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FSM>());
  rclcpp::shutdown();
  return 0;
}
