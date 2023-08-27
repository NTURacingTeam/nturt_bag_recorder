# NTURT bag recorder

## Introduction

ROS2 provides a command `ros2 bag record` to record messages from topics. And it in terms create `ros2bage_record` node for hendling it. However, the configuration isn't that easy to setup. Hence in this package, a node is provided to wrap around it and no extra configuration is needed.

## Usage

### Running the node

You can run this node by

```shell=
ros2 run nturt_bag_recorder nturt_bag_recorder_node.py
```

The recorded bags will appear in `~/.ros/bags/`, and the name of the bag is the time when the bag is recorded. If for some reason two bags are recorded at the same time, the latter one's name will be appended with a number, incrementing from 1.

### bag_decoder

This package also provides utilities to convert the recorded bag into csv files.

Usage:

```shell=
ros2 run bag_decoder [OPTIONS] \<bag\>
```

> Note: The bag passed to bag_decoder should be first merged using `merge_decode_bag.sh` with `-m` option (see: [merge_decode_bag](#merge_decode_bagsh)).

> Note: For more inofrmation about the usage of the utilities, use `-h` option to see the help message.

### merge_decode_bag.sh

This script can be used to `merge` and `decode` one or more recorded bag.

Usage:

```shell=
ros2 run merge_decode_bag.sh [OPTIONS] \<-d|-m\> \<INPUT_BAG1\> [INPUT_BAG2 ...]
```

#### merge

In case of sudden system crash causing the recorded bag to be corrupted, bag are splited into 10s segments. Hence this shell scripts first reindex them in case `metadata.yaml` file is lost and then merge them into a single one for further manipulation.

To use this mode, use `-m` option.

#### decode

Since the recorded bag has to be first porcessed by `merge_decode_bag.sh` with `-m` option, and then decoded by `bag_decoder`, this option can streamline the process by doing both at the same time. (Or parallelizing the decoding process if you were only using the `-d` option.)

To use this mode, use `-d` option.

> Note: For more inofrmation about the usage of the utilities, use `-h` option to see the help message.
