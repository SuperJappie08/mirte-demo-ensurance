#include <iostream>
#include <vector>
#include <algorithm>
#include "std_srvs/srv/set_bool.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace BT {

using SetBool = std_srvs::srv::SetBool;

class ToggleGripperService : public RosServiceNode<SetBool>
{
public:

    static constexpr const char * GRASP = "in_to_grasp";

    ToggleGripperService(const std::string & instance_name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params) :
        RosServiceNode<SetBool>(instance_name, conf, params)

    {
    }

    static PortsList providedPorts()
    {
        PortsList base_ports = RosServiceNode::providedPorts();

        PortsList child_ports = { 
                  InputPort<bool>(GRASP),
                };

        child_ports.merge(base_ports);

        return child_ports;
    }

    bool setRequest(typename Request::SharedPtr& request) override
    {
        bool to_grasp;
        getInput(GRASP,to_grasp);
        request->data = to_grasp;
        RCLCPP_INFO(logger(), "Sending request to toggle gripper");
        return true;
    }

    BT::NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "Failed to ToggleGripper Error: %d", error);
        return NodeStatus::FAILURE;
    }

    BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override
    {
        RCLCPP_INFO(logger(), "ToggleGripper response received");

        return response.get()->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

CreateRosNodePlugin(ToggleGripperService, "ToggleGripper");

}
