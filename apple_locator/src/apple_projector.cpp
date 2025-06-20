#include <apple_locator/apple_projector.hpp>
#include <rclcpp/node_options.hpp>

namespace apple_locator {

AppleProjector::AppleProjector(const rclcpp::NodeOptions & options) : Node("apple_projector", options){}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(apple_locator::AppleProjector)
