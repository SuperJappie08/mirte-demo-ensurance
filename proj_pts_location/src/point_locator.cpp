#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <mutex>
#include <limits>
#include <cmath>

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

class PointLocator : public rclcpp::Node {
public:
    PointLocator() : rclcpp::Node("point_locator_node"),
                     tf_buffer_(this->get_clock()),
                     tf_listener_(tf_buffer_) {
        using std::placeholders::_1;

        loadIntrinsicsFromYAML();

        // Subscribers - Publishers

        bbox_sub_ = this->create_subscription<sensor_msgs::msg::RegionOfInterest>(
            "/apple_detection/bbox_coords", 10,
            std::bind(&PointLocator::bboxCallback, this, _1));

        point_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/point_3D", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    
    }

    void init() {
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&PointLocator::depthCallback, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr bbox_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

    std::mutex mutex_;
    cv::Mat depth_image_;
    sensor_msgs::msg::RegionOfInterest latest_bbox_;
    bool got_depth_info_ = false;
    bool got_color_info_ = false;
    bool got_bbox_ = false;
    bool got_depth_ = false;
    bool wait_for_depth_ = false;

    // Camera Intrinsic Parameters
    float fx_d_, fy_d_, cx_d_, cy_d_;
    float fx_c_, fy_c_, cx_c_, cy_c_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Load camera intrinsics from yaml
    void loadIntrinsicsFromYAML() {
        std::string package_share = ament_index_cpp::get_package_share_directory("proj_pts_location");
        std::string config_path = package_share + "/config/camera_intrinsics.yaml";

        try {
            YAML::Node config = YAML::LoadFile(config_path);
            auto K_color = config["camera_intrinsics"]["K_color"].as<std::vector<double>>();
            auto K_depth = config["camera_intrinsics"]["K_depth"].as<std::vector<double>>();

            fx_c_ = K_color[0];
            fy_c_ = K_color[4];
            cx_c_ = K_color[2];
            cy_c_ = K_color[5];

            fx_d_ = K_depth[0];
            fy_d_ = K_depth[4];
            cx_d_ = K_depth[2];
            cy_d_ = K_depth[5];

            got_color_info_ = true;
            got_depth_info_ = true;

            RCLCPP_INFO(this->get_logger(), "Loaded intrinsics from YAML.");

        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load intrinsics: %s", e.what());
        }
    }

    // Receive bounding boxes
    void bboxCallback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_bbox_ = *msg;
        got_bbox_ = true;
        wait_for_depth_ = true;
        RCLCPP_INFO(get_logger(), "BBox received: x=%u y=%u w=%u h=%u",
                    latest_bbox_.x_offset, latest_bbox_.y_offset, latest_bbox_.width, latest_bbox_.height);
    }

    // Receive depth camera image
    void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!wait_for_depth_) {
            return;
        }

        try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
            if (cv_ptr->image.type() == CV_16UC1) {
                cv_ptr->image.convertTo(depth_image_, CV_32FC1, 1.0 / 1000.0);  // mm to meters
            } else if (cv_ptr->image.type() == CV_32FC1) {
                depth_image_ = cv_ptr->image;
            } else {
                RCLCPP_WARN(get_logger(), "Unexpected depth image type: %d", cv_ptr->image.type());
                return;
            }
            got_depth_ = true;
            wait_for_depth_ = false;
            compute3DPoint(msg->header.stamp);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    // Function to compute the 3D location
    void compute3DPoint(const rclcpp::Time& depth_stamp) {
        if (!(got_depth_info_ && got_color_info_ && got_depth_ && got_bbox_)) return;

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform("camera_rgb_optical_frame",
                                                   "camera_depth_optical_frame",
                                                   tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Transform lookup failed: %s", ex.what());
            return;
        }

        // ROI Region
        int u_center = latest_bbox_.x_offset + latest_bbox_.width / 2;
        int v_center = latest_bbox_.y_offset + latest_bbox_.height / 2;

        int width_offset = latest_bbox_.width / 4;  
        int height_offset = latest_bbox_.height / 4; 

        int x_min = u_center - width_offset;
        int x_max = u_center + width_offset;
        int y_min = v_center - height_offset;
        int y_max = v_center + height_offset;

        float sum_x = 0, sum_y = 0, sum_z = 0;
        int count = 0;

        for (int v = 0; v < depth_image_.rows; ++v) {
            for (int u = 0; u < depth_image_.cols; ++u) {
                float Z = depth_image_.at<float>(v, u);
                if (!std::isfinite(Z) || Z <= 0.0f || Z >= 20.0) continue;

                // Back-projetion in depth camera optical frame
                float X_d = (u - cx_d_) * Z / fx_d_;
                float Y_d = (v - cy_d_) * Z / fy_d_;

                // Transform to color camera optical frame
                tf2::Vector3 pt_d(X_d, Y_d, Z);
                tf2::Transform tf;
                tf2::fromMsg(transform.transform, tf);
                tf2::Vector3 pt_c = tf * pt_d;

                // Project to color image
                float u_c = fx_c_ * pt_c.x() / pt_c.z() + cx_c_;
                float v_c = fy_c_ * pt_c.y() / pt_c.z() + cy_c_;

                // Check if point in ROI
                if (u_c < x_min || u_c >= x_max || v_c < y_min || v_c >= y_max)
                    continue;

                sum_x += pt_c.x();
                sum_y += pt_c.y();
                sum_z += pt_c.z();
                count++;
            }
        }

        if (count > 0) {
            float avg_x = (sum_x / count) + 0.04;
            float avg_y = sum_y / count;
            float avg_z = (sum_z / count) - 0.02;

            geometry_msgs::msg::PointStamped point_msg_color_cam;
            point_msg_color_cam.header.frame_id = "camera_rgb_optical_frame";  
            point_msg_color_cam.header.stamp = depth_stamp;
            point_msg_color_cam.point.x = avg_x;
            point_msg_color_cam.point.y = avg_y;
            point_msg_color_cam.point.z = avg_z;
            
            geometry_msgs::msg::PointStamped point_msg_map;
            
            // Transform to map frame
            if (tf_buffer_.canTransform("map",
                                        point_msg_color_cam.header.frame_id,
                                        point_msg_color_cam.header.stamp,
                                        tf2::durationFromSec(5.0))) {
                try {
                    point_msg_map = tf_buffer_.transform(point_msg_color_cam, "map");
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(get_logger(), "Transform failed: %s", ex.what());
                    return;
                }
            } else {
                RCLCPP_WARN(get_logger(), "Transform to 'map' not available");
                return;
            }

            point_pub_->publish(point_msg_map.point);

            // Publish rviz marker
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.ns = "points";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = point_msg_map.point;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05; 
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            marker_pub_->publish(marker);

            RCLCPP_INFO(get_logger(), "Avg 3D point in map frame: [%.3f, %.3f, %.3f] from %d points",
                    point_msg_map.point.x,
                    point_msg_map.point.y,
                    point_msg_map.point.z,
                    count);
        }
        else {
            RCLCPP_WARN(get_logger(), "No valid 3D points found in inner bounding box.");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointLocator>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
