#pragma once
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/camera_subscriber.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>

namespace apple_locator {

class AppleProjector : public rclcpp::Node {
public:
  AppleProjector(const rclcpp::NodeOptions &options);

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cam_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pick_point_pub_;

  rclcpp::Subscription<vision_msgs::msg::BoundingBox2D>::SharedPtr
      detections_sub_;

  std::string depth_image_topic_;
  image_transport::CameraSubscriber depth_cam_sub_;

  std::mutex depth_lock_;
  sensor_msgs::msg::Image::ConstSharedPtr depth_img_ = nullptr;
  image_geometry::PinholeCameraModel depth_model_;
  image_geometry::PinholeCameraModel cb_depth_model_;

  void depth_cam_image_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr &image,
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info);

  void
  bbox_callback(const vision_msgs::msg::BoundingBox2D::ConstSharedPtr &bbox);
};

} // namespace apple_locator
