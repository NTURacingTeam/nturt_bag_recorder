#include "nturt_bag_recorder/bag_decoder_arg_parser.hpp"

// glibc include
#include <getopt.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#define COLOR_REST "\033[0m"
#define HIGHLIGHT "\033[0;1m"
#define COLOR_RED "\033[1;31m"

static const char *option = "ho:";
struct option long_option[] = {
    {"help", 0, NULL, 'h'},
    {"output", 1, NULL, 'o'},
    {0, 0, 0, 0},
};

static const char *usage =
    "TODO\n"
    "\n"
    "Usage: ./bag_decoder [OPTIONS] INPUT_BAG\n"
    "Options:\n"
    "    -h --help       Print this help message and exit\n"
    "    -o --output     Output directory, default to decoded_INPUT_BAG\n";

void parse_arg(int argc, char **argv, BagDecoderArg *arg) {
  // prevent getopt to print error message to stderr
  opterr = 0;

  while (1) {
    int opt = getopt_long(argc, argv, option, long_option, NULL);

    if (opt == -1) {
      break;
    }

    switch (opt) {
      case 'h':
        printf("%s\n", usage);
        exit(0);
        break;

      case 'o':
        arg->output_directory = optarg;
        break;

      case '?':
        fprintf(stderr,
                COLOR_RED "Error:" HIGHLIGHT " Unknown option: %c\n" COLOR_REST,
                optopt);
        printf("%s\n", usage);
        exit(1);
        break;
    }
  }

  if (argc == optind) {
    fprintf(stderr, COLOR_RED
            "Error:" HIGHLIGHT
            " No input bag given, expect exactly one bag\n" COLOR_REST);
    printf("%s\n", usage);
    exit(1);
  } else if (argc > optind + 1) {
    fprintf(stderr, COLOR_RED
            "Error:" HIGHLIGHT
            " Too many input bags given, expect exactly one bag\n" COLOR_REST);
    printf("%s\n", usage);
    exit(1);
  } else {
    arg->bag_file = argv[optind];

    // remove trailing slash if there is one
    if (arg->bag_file.back() == '/') {
      arg->bag_file.pop_back();
    }
  }

  if (arg->output_directory.empty()) {
    arg->output_directory = "decoded_" + arg->bag_file;
  }
}
