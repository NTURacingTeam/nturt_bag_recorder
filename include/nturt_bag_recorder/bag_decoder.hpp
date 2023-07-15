/**
 * @file bag_decoder.hpp
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for decoding bag file to csv.
 */

#ifndef BAG_DECODER_HPP
#define BAG_DECODER_HPP

// std include
#include <fstream>

// nturt include
#include "nturt_bag_recorder/bag_decoder_arg_parser.hpp"
#include "nturt_bag_recorder/csv_writer.hpp"

/**
 * @author QuantumSpawner jet22854111@gmail.com
 * @brief Class for decoding bag file to csv.
 */
class BagDecoder {
 public:
  /// @brief Constructor of BagDecoder.
  BagDecoder(BagDecoderArg arg);

  /// @brief Start decoding bag file to csv.
  void run();

 private:
  /// @brief Struct containing command line arguments.
  BagDecoderArg arg_;

  /// @brief Writer for csv file.
  Writer csv_writer_;
};

#endif  // BAG_DECODER_HPP
