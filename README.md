# NTURT bag recorder

## Introduction

ROS2 provides a command `ros2 bag record` to record messages from topics. And it in terms create `ros2bage_record` node for hendling it. However, the configuration isn't that easy to setup. Hence in this package, a node is provided to wrap around it and no extra configuration is needed.

## Usage

### Running the node

You can run this node by

```shell=
ros2 run nturt_bag_recorder nturt_bag_recorder_node.py
```

The recorded bags will appear in `~/.ros/bags/`, and the name of the bag is the time when the bag is recorded.

### merge_bag.sh

In case of sudden system crash causing the recorded bag to be corrupted, bag are splited into 10s segments. Hence this shell scripts first reindex them in case `metadata.yaml` file is lost and then merge them into a single one for further manipulation.

Usage:

```shell=
ros2 run merge_bag.sh [OPTIONS] \<bag\>
```

### bag_decoder

This package also provides utilities to convert the recorded bag into csv files.

Usage:

```shell=
ros2 run bag_decoder [OPTIONS] \<bag\>
```

> Note: The bag passed to bag_decoder should be first merged using `merge_bag.sh`.

> Note: For more inofrmation about the usage of the utilities, use `-h` option to see the help message.
