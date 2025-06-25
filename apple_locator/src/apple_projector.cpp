#include <functional>
#include <mutex>

#include <apple_locator/apple_projector.hpp>

#include <rclcpp/node_options.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <image_proc/utils.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/transport_hints.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

namespace apple_locator {

AppleProjector::AppleProjector(const rclcpp::NodeOptions &options)
    : Node("apple_projector", options), tf_buffer_(this->get_clock()),
      // NOTE: Merging TF node breaks IPC pre Jazzy
      tf_listener_(tf_buffer_ /*, this*/) {
  using namespace std::placeholders;
  this->declare_parameter<std::string>("image_transport", "raw");
  this->declare_parameter<double>("tf2_timeout", 1.0);
  this->declare_parameter<std::string>("target_frame", "map");

  // For compressed topics to remap appropriately, we need to pass a
  // fully expanded and remapped topic name to image_transport
  auto node_base = this->get_node_base_interface();
  this->depth_image_topic_ = node_base->resolve_topic_or_service_name(
      "camera/depth/image_rect", false);

  image_transport::TransportHints hints(this);
  auto qos = image_proc::getTopicQosProfile(this, this->depth_image_topic_);
  this->depth_cam_sub_ = image_transport::create_camera_subscription(
      this, depth_image_topic_,
      std::bind(&AppleProjector::depth_cam_image_callback, this, _1, _2),
      hints.getTransport(), qos);

  this->cam_point_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          "~/pick_point_camera",
          rclcpp::SystemDefaultsQoS().durability_volatile());
  this->pick_point_pub_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          "pick_point", rclcpp::SystemDefaultsQoS().durability_volatile());

  this->detections_sub_ =
      this->create_subscription<vision_msgs::msg::BoundingBox2D>(
          "/apple_detector/bbox_coords", rclcpp::SystemDefaultsQoS(),
          std::bind(&AppleProjector::bbox_callback, this, _1));
}

void AppleProjector::depth_cam_image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr &image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info) {
  std::scoped_lock lock(this->depth_lock_);
  this->depth_img_ = image;
  this->depth_model_.fromCameraInfo(info);
}

void AppleProjector::bbox_callback(
    const vision_msgs::msg::BoundingBox2D::ConstSharedPtr &bbox) {
  if (!depth_img_) {
    RCLCPP_WARN(get_logger(), "No depth image recieved yet");
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  image_geometry::PinholeCameraModel cam_model;
  try {
    std::scoped_lock lock(depth_lock_);
    cv_ptr = cv_bridge::toCvCopy(depth_img_);
    cam_model = image_geometry::PinholeCameraModel(depth_model_);
    if (cv_ptr->image.type() != CV_32FC1) {
      RCLCPP_WARN(get_logger(), "Unexpected depth image type: %d",
                  cv_ptr->image.type());
      return;
    }
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // ROI Region
  auto u_center = bbox->center.position.x;
  auto v_center = bbox->center.position.y;

  auto width_offset = bbox->size_x / 2;
  auto height_offset = bbox->size_y / 2;

  auto x_min = u_center - width_offset;
  auto x_max = u_center + width_offset;
  auto y_min = v_center - height_offset;
  auto y_max = v_center + height_offset;

  double sum_x = 0, sum_y = 0, sum_z = 0;
  int count = 0;

  for (int v = 0; v < cv_ptr->image.rows; ++v) {
    for (int u = 0; u < cv_ptr->image.cols; ++u) {
      float z = cv_ptr->image.at<float>(v, u);
      if (!std::isfinite(z) || z <= 0.0f || z >= 20.0)
        continue;
      // Check if point in ROI
      if (u < x_min || u >= x_max || v < y_min || v >= y_max)
        continue;

      auto ray = cam_model.projectPixelTo3dRay(cv::Point2d(u, v));

      sum_x += ray.x * z;
      sum_y += ray.y * z;
      sum_z += ray.z * z;
      count++;
    }
  }

  if (count > 0) {
    // FIXME(SuperJappie08): There were some offsets here. -> Probably because
    // of bad camera config
    auto avg_x = (sum_x / count); // + 0.04;
    auto avg_y = (sum_y / count);
    auto avg_z = (sum_z / count); // - 0.02;

    geometry_msgs::msg::PointStamped point_msg_cam;
    point_msg_cam.header.frame_id = cam_model.tfFrame();
    point_msg_cam.header.stamp = cam_model.stamp();
    point_msg_cam.point.x = avg_x;
    point_msg_cam.point.y = avg_y;
    point_msg_cam.point.z = avg_z;

    RCLCPP_INFO(
        get_logger(),
        "Avg 3D point in camera frame: [%.3f, %.3f, %.3f] from %d points",
        point_msg_cam.point.x, point_msg_cam.point.y, point_msg_cam.point.z,
        count);

    cam_point_pub_->publish(point_msg_cam);

    geometry_msgs::msg::PointStamped point_msg_map;

    std::string target_frame = this->get_parameter("target_frame").as_string();
    try {
      point_msg_map = tf_buffer_.transform(
          point_msg_cam, target_frame,
          tf2::durationFromSec(this->get_parameter("tf2_timeout").as_double()));
    } catch (tf2::LookupException &ex) {
      RCLCPP_WARN(get_logger(), "Transform to '%s' not available: %s",
                  target_frame.c_str(), ex.what());
      return;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "Transform failed: %s", ex.what());
      return;
    }

    pick_point_pub_->publish(point_msg_map);

    RCLCPP_INFO(get_logger(),
                "Avg 3D point in '%s' frame: [%.3f, %.3f, %.3f] from %d points",
                target_frame.c_str(), point_msg_map.point.x,
                point_msg_map.point.y, point_msg_map.point.z, count);
  } else {
    RCLCPP_WARN(get_logger(),
                "No valid 3D points found in inner bounding box.");
  }
}

} // namespace apple_locator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(apple_locator::AppleProjector)
