#!/bin/bash

set -e

# the name of the bag is the timestemp when the bag is recorded
BAG_NAME="$(date +'%Y-%m-%d-%I-%M-%S')"

# start recording the bag
ros2 bag record -o ${BAG_NAME} /from_can_bus /rosout /system_stats
