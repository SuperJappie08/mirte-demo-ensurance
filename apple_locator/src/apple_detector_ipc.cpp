#include <apple_locator/apple_detector_ipc.hpp>

#include <chrono>
#include <cstdint>
#include <functional>

#include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.hpp>
// #include <image_transport/subscriber.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/wait_for_message.hpp>
// #include <sensor_msgs/msg/region_of_interest.hpp>
#include <create_aabb.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <image_proc/utils.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace apple_locator {

AppleDetectorIPC::AppleDetectorIPC(const rclcpp::NodeOptions &node_options)
    : Node("apple_detector", node_options) {
  using namespace std::placeholders;

  this->declare_parameter<std::string>("image_transport", "raw");
  this->declare_parameter("red_min", std::vector<int64_t>{20, 150, 100});
  this->declare_parameter("red_max", std::vector<int64_t>{190, 255, 255});

  // For compressed topics to remap appropriately, we need to pass a
  // fully expanded and remapped topic name to image_transport
  auto node_base = this->get_node_base_interface();
  this->image_topic_ = node_base->resolve_topic_or_service_name("image", false);

  image_transport::TransportHints hints(this);
  auto qos = image_proc::getTopicQosProfile(this, this->image_topic_);
  this->image_sub_ = image_transport::create_subscription(
      this, image_topic_,
      std::bind(&AppleDetectorIPC::image_callback, this, _1),
      hints.getTransport(), qos);

  this->get_point_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "get_point",
      std::bind(&AppleDetectorIPC::get_point_srv_callback, this, _1, _2));

  this->bounding_box_pub_ =
      // this->create_publisher<sensor_msgs::msg::RegionOfInterest>(
      this->create_publisher<vision_msgs::msg::BoundingBox2D>(
          "~/bbox_coords", rclcpp::SystemDefaultsQoS().durability_volatile());
}

void AppleDetectorIPC::get_point_srv_callback(
    const GetPointSrv::Request::ConstSharedPtr /*request*/,
    GetPointSrv::Response::SharedPtr response) {
  RCLCPP_INFO(get_logger(), "Attempting to retrieve point");

  if (!this->img_) {
    RCLCPP_ERROR(get_logger(), "Failed to retrieve image.");
    response->success = false;
    response->message = "Failed to retrieve image";
    return;
  }

  // sensor_msgs::msg::Image msg;

  // auto timeout = get_parameter("image_timeout").as_int();
  // if (!rclcpp::wait_for_message(
  //   msg, shared_from_this(), "image",
  //   std::chrono::duration<uint64_t, std::milli>(timeout))) {
  //   RCLCPP_ERROR(get_logger(), "Failed to retrieve image within %ld ms.",
  //                timeout);
  //   response->success = false;
  //   response->message = "Failed to retrieve image within the timeout";
  //   return;
  // }

  auto image = cv_bridge::toCvCopy(this->img_, "bgr8");

  cv::medianBlur(image->image, image->image, 3);
  cv::Mat image_lab;
  cv::cvtColor(image->image, image_lab, cv::COLOR_BGR2Lab);
  cv::Mat mask;

  auto min_array = get_parameter("red_min").as_integer_array();
  auto max_array = get_parameter("red_max").as_integer_array();

  if (min_array.size() < 3) {
    RCLCPP_WARN(get_logger(),
                "parameter red_min should have size 3, using default.");
    min_array = {20, 150, 100};
  }
  if (max_array.size() < 3) {
    RCLCPP_WARN(get_logger(),
                "parameter red_max should have size 3, using default.");
    max_array = {190, 255, 255};
  }

  auto red_min = cv::Scalar(min_array[0], min_array[1], min_array[2]);
  auto red_max = cv::Scalar(max_array[0], max_array[1], max_array[2]);

  cv::inRange(image_lab, red_min, red_max, mask);

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

  RCLCPP_INFO(get_logger(), "Succes %ld", circles.size());

  bounding_box_pub_->publish(vision_msgs::createAABB2D(
      circles[0][0] - circles[0][2], circles[0][1] - circles[0][2],
      2 * circles[0][2], 2 * circles[0][2]));
  response->success = true;
}

void AppleDetectorIPC::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  this->img_ = msg;
}

} // namespace apple_locator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(apple_locator::AppleDetectorIPC)
