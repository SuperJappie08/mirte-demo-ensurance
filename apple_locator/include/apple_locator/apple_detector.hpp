#pragma once

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
// #include <sensor_msgs/msg/region_of_interest.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace apple_locator {

class AppleDetector : public rclcpp::Node {
public:
  using GetPointSrv = std_srvs::srv::Trigger;
  // explicit 
  AppleDetector(const rclcpp::NodeOptions &node_options);
  // ~UsbCamNode();

private:
  // image_transport::ImageTransport it_;

  // rclcpp::Publisher<sensor_msgs::msg::RegionOfInterest>::SharedPtr
  rclcpp::Publisher<vision_msgs::msg::BoundingBox2D>::SharedPtr
      bounding_box_pub_ = nullptr;
  // image_transport::
  rclcpp::Service<GetPointSrv>::SharedPtr get_point_srv_ = nullptr;

  void
  get_point_srv_callback(const GetPointSrv::Request::ConstSharedPtr request,
                         GetPointSrv::Response::SharedPtr response);
};

} // namespace apple_locator
