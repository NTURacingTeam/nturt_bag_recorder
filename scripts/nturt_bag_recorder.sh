#!/bin/bash

set -e

# the name of the bag is the timestemp when the bag is recorded
BAG_NAME="$(date +'%Y-%m-%d-%H-%M-%S')"

# start recording the bag
# the flags are:
# -o: the name of the bag
# -d: duration in seconds before the bagfile will be split
# --storage-preset-profile: configuration preset for storage, resilient to 
#   reduce the chance of data corruption in case of a system crash
ros2 bag record -o ${BAG_NAME} \
    -d 10 \
    --storage-preset-profile resilient \
    /from_can_bus /gps_fix /gps_vel /system_stats /rosout
