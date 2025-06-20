#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "fsm/action/move_arm.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include "ik_solver.hpp"

// TF2 includes
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class MoveArmServer : public rclcpp::Node
{
public:
    using MoveArm = fsm::action::MoveArm;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MoveArm>;

    MoveArmServer() : Node("move_arm_server")
    {
        // Initialize TF2 buffer and listener using direct construction
        try {
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            RCLCPP_INFO(this->get_logger(), "TF2 buffer and listener initialized successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize TF2: %s", e.what());
            throw;
        }

        action_server_ = rclcpp_action::create_server<MoveArm>(
            this,
            "move_arm",
            std::bind(&MoveArmServer::handle_goal, this, _1, _2),
            std::bind(&MoveArmServer::handle_cancel, this, _1),
            std::bind(&MoveArmServer::handle_accepted, this, _1));

        joint_trajectory_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/mirte_master_arm_controller/joint_trajectory", 10);

        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MoveArmServer::joint_state_callback, this, std::placeholders::_1));

        target_joints_ = {
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_joint",
            "shoulder_pan_joint"
        };

        RCLCPP_INFO(this->get_logger(), "MoveArmServer initialized successfully");
    }

private:
    rclcpp_action::Server<MoveArm>::SharedPtr action_server_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    std::shared_ptr<MoveArm::Feedback> feedback_;

    std::unordered_map<std::string, double> current_joint_positions_;
    std::unordered_set<std::string> target_joints_;
    std::mutex mutex_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const MoveArm::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal (map frame): x=%.2f, y=%.2f, z=%.2f", goal->x, goal->y, goal->z);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{std::bind(&MoveArmServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        auto result = std::make_shared<MoveArm::Result>();

        double x = goal_handle->get_goal()->x;
        double y = goal_handle->get_goal()->y;
        double z = goal_handle->get_goal()->z;

        if (goal_handle->get_goal()->home == true){
            std::vector<double> reset = {0.0,0.8,-1.5,-0.8};
            send_joint_trajectory(reset);
            result->success = true;
            result->message = "Homing.";
            goal_handle->succeed(result);
            return;
        }

        geometry_msgs::msg::PointStamped map_point;
        map_point.header.frame_id = "map";
        map_point.header.stamp = this->get_clock()->now();
        map_point.point.x = x;
        map_point.point.y = y;
        map_point.point.z = z;

        geometry_msgs::msg::PointStamped base_link_point;

        // Check if tf_buffer_ is valid before using it
        if (!tf_buffer_) {
            RCLCPP_ERROR(this->get_logger(), "TF buffer not initialized!");
            result->success = false;
            result->message = "TF buffer not initialized";
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting transform lookup...");

        // Wait for transform to become available with timeout
        try {
            std::string timeout_msg;
            if (!tf_buffer_->canTransform("base_link", "map", tf2::TimePointZero, tf2::durationFromSec(5.0), &timeout_msg)) {
                RCLCPP_WARN(this->get_logger(), "Transform not available: %s", timeout_msg.c_str());
                result->success = false;
                result->message = "Transform to 'base_link' not available: " + timeout_msg;
                goal_handle->abort(result);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Transform is available, performing transformation...");
            
            // Perform the transform
            base_link_point = tf_buffer_->transform(map_point, "base_link", tf2::durationFromSec(1.0));
            x = base_link_point.point.x;
            y = base_link_point.point.y;
            z = base_link_point.point.z;

            RCLCPP_INFO(this->get_logger(), "Transformed to base_link: x=%.2f, y=%.2f, z=%.2f", x, y, z);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
            result->success = false;
            result->message = "Transform failed: " + std::string(ex.what());
            goal_handle->abort(result);
            return;
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Unexpected error during transform: %s", ex.what());
            result->success = false;
            result->message = "Unexpected error during transform: " + std::string(ex.what());
            goal_handle->abort(result);
            return;
        }

        IkSolver ik_solver;
        std::vector<double> target_joint_angles = ik_solver.calculateAngles(x, y, z);

        RCLCPP_INFO(this->get_logger(), "Calculated joint angles: %.2f, %.2f, %.2f, %.2f",
                    target_joint_angles[0], target_joint_angles[1],
                    target_joint_angles[2], target_joint_angles[3]);

        feedback_ = std::make_shared<MoveArm::Feedback>();
        feedback_->status = "Goal joint angles: " +
                            std::to_string(target_joint_angles[0]) + " " +
                            std::to_string(target_joint_angles[1]) + " " +
                            std::to_string(target_joint_angles[2]) + " " +
                            std::to_string(target_joint_angles[3]);
        goal_handle->publish_feedback(feedback_);

        RCLCPP_INFO(this->get_logger(), "Executing arm movement...");
        send_joint_trajectory(target_joint_angles);

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Canceled.";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                return;
            }

            auto joint_positions = get_current_joint_positions();

            if (!joint_positions.empty()) {
                double shoulder = joint_positions["shoulder_lift_joint"];
                double elbow = joint_positions["elbow_joint"];
                double wrist = joint_positions["wrist_joint"];
                double pan = joint_positions["shoulder_pan_joint"];

                double total_error = fabs(pan - target_joint_angles[0]) +
                                     fabs(shoulder - target_joint_angles[1]) +
                                     fabs(elbow - target_joint_angles[2]) +
                                     fabs(wrist - target_joint_angles[3]);

                if (total_error < 0.05) {
                    result->success = true;
                    result->message = "Goal reached.";
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Arm moved successfully.");
                    return;
                }

                feedback_->status = "Tracking error: " + std::to_string(total_error);
                goal_handle->publish_feedback(feedback_);
            }

            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }

        result->success = false;
        result->message = "Movement failed.";
        goal_handle->abort(result);
        RCLCPP_INFO(this->get_logger(), "Arm movement failed.");
    }

    void send_joint_trajectory(const std::vector<double> &joint_angles)
    {
        trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
        trajectory_msgs::msg::JointTrajectoryPoint point;

        joint_trajectory_msg.joint_names = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"};

        point.positions = joint_angles;
        point.time_from_start.sec = 5;

        joint_trajectory_msg.points.push_back(point);
        joint_trajectory_pub_->publish(joint_trajectory_msg);
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        for (size_t i = 0; i < msg->name.size(); ++i) {
            const std::string &joint_name = msg->name[i];
            if (target_joints_.count(joint_name) && i < msg->position.size()) {
                current_joint_positions_[joint_name] = msg->position[i];
            }
        }
    }

    std::unordered_map<std::string, double> get_current_joint_positions()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_joint_positions_;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveArmServer>());
    rclcpp::shutdown();
    return 0;
}