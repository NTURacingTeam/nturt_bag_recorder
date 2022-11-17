#!/bin/bash

# the name of the bag is the timestemp when the bag is recorded
BAG_NAME="$(date +'%d-%m-%Y_%Ih%Mm%S')"

# start recording the bag
rosbag record -O ${BAG_NAME} /received_messages /sent_messages /GPS_fix
