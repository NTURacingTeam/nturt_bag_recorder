# NTURT bag recorder

## Introduction

ROS provides a command `rosbag record -a` to record all messages from all topics. Yet it don't provide a node that does this job.
This ros package `nturt_bag_recorder` provide a node that can de used to record all messages when run.

## Usage

### Running the node

You can run this node by

```shell=
roslaunch nturt_bag_recorder nturt_bag_recorder.launch
```

The recorded bags will appear in `bags/` directory, and the name of the bag is the time when the bag is recorded.

### Configuring the node

Ypu may custom the record scripts and the directory where the bags will appear in the launch file `nturt_ros_bag_recorder.launch` in `launch/`.
Currently, those are set to `scripts/nturt_bag_recorder.sh` and `bags/` respectively.
