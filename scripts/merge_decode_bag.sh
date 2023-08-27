#!/bin/bash

COLOR_REST='\e[0m'
HIGHLIGHT='\e[0;1m'
REVERSE='\e[0;7m'
COLOR_GREEN='\e[0;32m'
COLOR_RED='\e[1;31m'

print_usage() {
    echo "Merge split bag into one (merge mode), and/or decode bag into csv file (decode mode)."
    echo "Mergeand or decode mode can be selected by -m or -d option, respectively. And both can be selected at the same time."
    echo "Supports processing multiple input bags into multiple output bags in parallel."
    echo ""
    echo "Usage: ./merge_decode_bag.sh [OPTIONS] <-d|-m> <INPUT_BAG1> [INPUT_BAG2 ...]"
    echo "Options:"
    echo "    -d --decode     Decode mode."
    echo "    -h --help       Print this help message and exit"
    echo "    -m --merge      Merge mode."
}

merge_decode_bag_helper() {
    # remove trailing slash if it has one
    local INPUT_BAG=${1%/}

    if [[ ${MERGE_MODE} == true ]]; then
        # remove directory path if it has one
        local OUTPUT_BAG="merged_${INPUT_BAG##*/}"

        # ros2 bag convert config file
        local CONFIG_FILE=$(mktemp -t --suffix=.yaml  merge_XXXXXXXX)

        echo "output_bags:
  - uri: ${OUTPUT_BAG}
    all: true" > ${CONFIG_FILE}

        ros2 bag reindex ${INPUT_BAG} &>/dev/null
        if [ $? != 0 ]; then
            echo -e "${COLOR_RED}Error: ${HIGHLIGHT}Failed to reindex ${INPUT_BAG}${COLOR_REST}" >&2
            exit 1
        fi

        ros2 bag convert -i ${INPUT_BAG} -o ${CONFIG_FILE} &>/dev/null
        if [ $? != 0 ]; then
            echo -e "${COLOR_RED}Error: ${HIGHLIGHT}Failed to convert ${INPUT_BAG}${COLOR_REST}" >&2
            exit 1
        fi

        echo "merged ${INPUT_BAG} to ${OUTPUT_BAG}"

        if [[ ${DECODE_MODE} == true ]]; then
            INPUT_BAG=${OUTPUT_BAG}
        fi
    fi

    if [[ ${DECODE_MODE} == true ]]; then
        # remove directory path if it has one
        local OUTPUT_BAG="decoded_${INPUT_BAG##*/}"

        ros2 run nturt_bag_recorder bag_decoder -o ${OUTPUT_BAG} ${INPUT_BAG} &>/dev/null
        if [ $? != 0 ]; then
            echo -e "${COLOR_RED}Error: ${HIGHLIGHT}Failed to decode ${INPUT_BAG}${COLOR_REST}" >&2
            exit 1
        fi

        echo "decoded ${INPUT_BAG} to ${OUTPUT_BAG}"
    fi
    
}

# default argument
MERGE_MODE=false
DECODE_MODE=false

PARAM=$(getopt -o dhm -l decode,help,merge -n "$0" -- "$@")

if [ $? != 0 ]; then
    echo -e "${COLOR_RED}Error: ${HIGHLIGHT}Unknown option${COLOR_REST}" >&2
    print_usage
    exit 1
fi

set -e

eval set -- "${PARAM}"

while true; do
    case "$1" in
        -d|--decode)
            DECODE_MODE=true
            shift
            ;;

        -h|--help)
            print_usage
            exit
            shift
            ;;

        -m|--merge)
            MERGE_MODE=true
            shift
            ;;

        # ros2 will automatically pass this option in when launch
        --ros-args)
            shift
            ;;
        
        --)
            shift
            if [[ $# == 0 ]]; then
                echo -e "${COLOR_RED}Error: ${HIGHLIGHT}No input bag given, expect at least one bag${COLOR_REST}" >&2
                print_usage
                exit 1
            elif [[ ${DECODE_MODE} == false && ${MERGE_MODE} == false ]]; then
                echo -e "${COLOR_RED}Error: ${HIGHLIGHT}No mode selected, expect -d or -m option${COLOR_REST}" >&2
                print_usage
                exit 1
            fi
            export DECODE_MODE MERGE_MODE
            export -f merge_decode_bag_helper
            parallel merge_decode_bag_helper ::: $@
            break
            ;;
    esac
done
