#include <apple_locator/apple_detector.hpp>

#include <chrono>
#include <cstdint>
#include <functional>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/wait_for_message.hpp>
// #include <sensor_msgs/msg/region_of_interest.hpp>
#include <create_aabb.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace apple_locator {

AppleDetector::AppleDetector(const rclcpp::NodeOptions &node_options)
    : Node("apple_detector", node_options) {
  using namespace std::placeholders;

  this->declare_parameter("image_timeout", 1000);

  this->get_point_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "get_point",
      std::bind(&AppleDetector::get_point_srv_callback, this, _1, _2));

  this->bounding_box_pub_ =
      // this->create_publisher<sensor_msgs::msg::RegionOfInterest>(
      this->create_publisher<vision_msgs::msg::BoundingBox2D>(
          "~/bbox_coords", rclcpp::SystemDefaultsQoS().durability_volatile());
}

void AppleDetector::get_point_srv_callback(
    const GetPointSrv::Request::ConstSharedPtr /*request*/,
    GetPointSrv::Response::SharedPtr response) {
  RCLCPP_INFO(get_logger(), "Attempting to retrieve point");

  auto options = rclcpp::SubscriptionOptions();
  // options
  // image_transport::Subscriber sub = it_.subscribe(
  //     "image_raw", 1,
  //     [](const sensor_msgs::msg::Image::ConstSharedPtr &msg) { (void)msg; },
  //     nullptr, options);

  sensor_msgs::msg::Image msg;
  // rclcpp::wait_for_message(&msg, sub, this->get_node_options().context());

  auto timeout = get_parameter("image_timeout").as_int();
  if (!rclcpp::wait_for_message(
          msg, shared_from_this(), "/image_raw",
          std::chrono::duration<uint64_t, std::milli>(timeout))) {
    RCLCPP_ERROR(get_logger(), "Failed to retrieve image within %ld ms.",
                 timeout);
    response->success = false;
    response->message = "Failed to retrieve image within the timeout";
    return;
  }

  auto image = cv_bridge::toCvCopy(msg, "bgr8");

  cv::medianBlur(image->image, image->image, 3);
  cv::Mat image_lab;
  cv::cvtColor(image->image, image_lab, cv::COLOR_BGR2Lab);
  cv::Mat mask;

  cv::inRange(image_lab, cv::Scalar(20, 150, 100), cv::Scalar(190, 255, 255),
              mask);

  cv::GaussianBlur(mask, mask, cv::Size_<int>(5, 5), 2);
  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(mask, circles, cv::HOUGH_GRADIENT, 1, mask.rows / 8, 100, 18,
                   5, 500);

  if (circles.empty()) {
    RCLCPP_ERROR(get_logger(), "No apples found");
    response->success = false;
    response->message = "No apples found!";
    return;
  }

  RCLCPP_INFO(get_logger(), "Succes");

  bounding_box_pub_->publish(vision_msgs::createAABB2D( circles[0][0]-circles[0][2], circles[0][1] - circles[0][2], 2*circles[0][2], 2*circles[0][2]));
  response->success = true;
}

} // namespace apple_locator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(apple_locator::AppleDetector)
