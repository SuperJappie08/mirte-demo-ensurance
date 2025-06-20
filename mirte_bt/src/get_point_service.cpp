#include <iostream>
#include <vector>
#include <algorithm>
#include "std_srvs/srv/trigger.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace BT {

using Trigger = std_srvs::srv::Trigger;

class GetPointService : public RosServiceNode<Trigger>
{
public:
    GetPointService(const std::string & instance_name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params) :
        RosServiceNode<Trigger>(instance_name, conf, params)

    {
    }

    static PortsList providedPorts()
    {
        PortsList base_ports = RosServiceNode::providedPorts();

        PortsList child_ports = { 
                };

        child_ports.merge(base_ports);

        return child_ports;
    }

    bool setRequest(typename Request::SharedPtr& request) override
    {
        RCLCPP_INFO(logger(), "Sending request to get point");
        return true;
    }

    BT::NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "Failed to GetPoint Error: %d", error);
        return NodeStatus::FAILURE;
    }

    BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override
    {
        RCLCPP_INFO(logger(), "GetPoint response received");

        return response.get()->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

BT_REGISTER_ROS_NODES(factory, params)
{
  RosNodeParams aug_params;
  aug_params.nh = params.nh;
  aug_params.server_timeout = std::chrono::milliseconds(40000);   //Nav2 can take a while to respond, especialy in a container.
  //TODO: options.use_global_arguments(false) need to fix this for plguins somehow. also applies to client name
  factory.registerNodeType<GetPointService>("RequestPoint", aug_params);
}

}
