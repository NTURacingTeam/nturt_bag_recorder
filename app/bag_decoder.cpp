// std include
#include <filesystem>
#include <memory>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// nturt include
#include "nturt_bag_recorder/bag_decoder.hpp"

int main(int argc, char **argv) {
  BagDecoderArg arg;
  parse_arg(argc, argv, &arg);

  create_directory(arg.output_directory);

  BagDecoder bad_decoder_node(arg);
  bad_decoder_node.run();

  return 0;
}
