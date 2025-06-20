#include "geometry_msgs/msg/point.hpp"
#include <cmath>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace BT {

using Point = geometry_msgs::msg::Point;


class ReceivePointSub : public RosTopicSubNode<Point>
{
    public:
    
    static constexpr const char* POINT_OUT = "out_point";

    ReceivePointSub(const std::string & instance_name,const NodeConfig &conf,const RosNodeParams& params)
        : RosTopicSubNode<Point>(instance_name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        PortsList base_ports = RosTopicSubNode::providedPorts();

        PortsList child_ports = {
                        OutputPort<Point>(POINT_OUT),
                };

        child_ports.merge(base_ports);

        return child_ports;
    }

  /** Callback invoked in the tick. You must return either SUCCESS of FAILURE
   *
   * @param last_msg the latest message received since the last tick.
   * it might be empty.
   * @return the new status of the Node, based on last_msg
   */
    BT::NodeStatus onTick(const typename Point::SharedPtr& last_msg) override
    {
        if(last_msg) {
            
            std::stringstream ss;
            ss << "Received 3D Point ";
            ss << "x : " << last_msg->x;
            ss << "y : " << last_msg->y;
            ss << "z : " << last_msg->z;
            RCLCPP_INFO(logger(), ss.str().c_str());

            setOutput(POINT_OUT, *last_msg);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
};

CreateRosNodePlugin(ReceivePointSub, "ReceivePoint");

}