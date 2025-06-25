#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>

#include <image_transport/subscriber.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>

namespace apple_locator {

class AppleDetectorIPC : public rclcpp::Node {
public:
  using GetPointSrv = std_srvs::srv::Trigger;
  // explicit
  AppleDetectorIPC(const rclcpp::NodeOptions &node_options);

private:
  image_transport::Subscriber image_sub_;

  rclcpp::Publisher<vision_msgs::msg::BoundingBox2D>::SharedPtr
      bounding_box_pub_ = nullptr;
  rclcpp::Service<GetPointSrv>::SharedPtr get_point_srv_ = nullptr;

  std::string image_topic_;
  sensor_msgs::msg::Image::ConstSharedPtr img_ = nullptr;

  void
  get_point_srv_callback(const GetPointSrv::Request::ConstSharedPtr request,
                         GetPointSrv::Response::SharedPtr response);

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
};

} // namespace apple_locator
