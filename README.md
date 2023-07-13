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
