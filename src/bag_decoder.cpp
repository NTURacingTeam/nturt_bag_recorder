#include "nturt_bag_recorder/bag_decoder.hpp"

// ros2 include
#include <rclcpp/rclcpp.hpp>

// std include
#include <functional>
#include <memory>

BagDecoder::BagDecoder(rclcpp::NodeOptions _options)
    : Node("nturt_bag_decoder", _options) {}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(BagDecoder)
