// std include
#include <memory>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// nturt include
#include "nturt_bag_recorder/bag_decoder.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  auto bad_decoder_node = std::make_shared<BagDecoder>(options);

  executor.add_node(bad_decoder_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
