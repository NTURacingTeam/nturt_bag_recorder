/**
 * @file bag_decoder.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief ROS2 package for
 */

#ifndef BAG_DECODER_HPP
#define BAG_DECODER_HPP

// std include
#include <memory>

// ros2 include
#include <rclcpp/rclcpp.hpp>

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for
 */
class BagDecoder : public rclcpp::Node {
 public:
  /// @brief Constructor of BagDecoder.
  BagDecoder(rclcpp::NodeOptions _options);

 private:
};

#endif  // BAG_DECODER_HPP
