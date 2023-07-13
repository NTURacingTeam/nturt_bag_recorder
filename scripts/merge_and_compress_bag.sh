#!/bin/bash

COLOR_REST='\e[0m'
HIGHLIGHT='\e[0;1m'
REVERSE='\e[0;7m'
COLOR_GREEN='\e[0;32m'
COLOR_RED='\e[1;31m'

print_usage() {
    echo "Merge and compress bag files"
    echo ""
    echo "Usage: ./merge_and_compress_bag.sh <INPUT_BAG> [OPTIONS]"
    echo "Options:"
    echo "    -h --help       Print this help message and exit"
    echo "    -o --output     Output bag file name, default to merged_INPUT_BAG"
}

# default arguments
OUTPUT_BAG=""

PARAM=$(getopt -o ho: -l help,output: -n "$0" -- "$@")

if [ $? != 0 ]; then
    print_usage
    exit 1
fi

set -e

eval set -- "${PARAM}"

while true; do
    case "$1" in
        -h|--help)
            print_usage
            exit
            shift
            ;;

        -o|--output)
            # remove trailing slash if it has one
            OUTPUT_BAG=${2%/}
            shift 2
            ;;

        # ros2 will automatically pass this option in when launch
        --ros-args)
            shift
            ;;
        
        --)
            shift
            if [[ -z $* ]]; then
                echo -e "${COLOR_RED}Error: ${HIGHLIGHT}No input bag given, expect exactly one bag${COLOR_REST}" >&2
                print_usage
                exit 1
            elif [[ $# != 1 ]]; then
                echo -e "${COLOR_RED}Error: ${HIGHLIGHT}More than argument given, expect only one bag${COLOR_REST}" >&2
                print_usage
                exit 1
            else
                # remove trailing slash if it has one
                INPUT_BAG=${1%/}
            fi
            break
            ;;
    esac
done

if [[ -z ${OUTPUT_BAG} ]]; then
    OUTPUT_BAG="merged_${INPUT_BAG##*/}"
fi

# ros2 bag convert config file
CONFIG_FILE=$(mktemp -t --suffix=.yaml  merge_and_compress_XXXXXXXX)

echo "output_bags:
  - uri: ${OUTPUT_BAG}
    all: true
    compression_mode: file
    compression_format: zstd" > ${CONFIG_FILE}

echo "reindexing ${INPUT_BAG}..."
ros2 bag reindex ${INPUT_BAG}

echo "merging and compressing ${INPUT_BAG}..."
ros2 bag convert -i ${INPUT_BAG} -o ${CONFIG_FILE}
