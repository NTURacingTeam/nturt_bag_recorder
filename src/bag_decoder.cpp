#include "nturt_bag_recorder/bag_decoder.hpp"

// std include
#include <fstream>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

// rosbag2 include
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

// nturt include
#include "nturt_bag_recorder/bag_decoder_arg_parser.hpp"

BagDecoder::BagDecoder(BagDecoderArg arg)
    : arg_(arg), csv_writer_(arg.output_file) {}

void BagDecoder::run() {
  std::vector<std::string> header = {"a", "b", "c"};
  csv_writer_.write_row(header);

  std::vector<int> data = {1, 2, 3};
  csv_writer_.write_row(data);
}
