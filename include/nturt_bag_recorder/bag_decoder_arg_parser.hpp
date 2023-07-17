#ifndef BAG_DECODER_ARG_PARSER_HPP
#define BAG_DECODER_ARG_PARSER_HPP

// std include
#include <string>

struct BagDecoderArg {
  /// @brief The input bag file.
  std::string bag_file;

  /// @brief The output directory.
  std::string output_directory;
};

/**
 * @brief Parse command line argument.
 *
 * @param argc The number of command line arguments.
 * @param argv The command line arguments.
 * @param arg The argument struct.
 */
void parse_arg(int argc, char **argv, BagDecoderArg *arg);

#endif  // BAG_DECODER_ARG_PARSER_HPP
