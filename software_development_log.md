# National Taiwan University Racing Team Software Development Log
###### tags: `development_log` `NTURT`
##### Group: electrical group
##### Person in charge: 羅紀翔
##### Authors: 羅紀翔
##### Subsystem: RPI
##### Subsystem number: RP2
##### Software name: bag_recorder
##### Repository: [github](https://github.com/NTURacingTeam/nturt_bag_recorder.git)
##### Started designing date: 2022/8/17
##### Current version: 1.0
##### Last modified date: 2022/8/28
---

## Engineering goal:

Record all ros messages from all topics for using in analysis in the future. Since ros does not provide a node for recording bags, a python wrapper to run `rosbag record -a` command is implemented here.

## Program structure:

Using python `subprocess` package to run `rosbag record -a` command to record a bag. And stop the command when shutdown.

## Included libraries:

- subprocess

## Testing environment:

- bash 5.0.17(1)-release (x86_64-pc-linux-gnu)
- ROS noetic

##### Testing hardware:

- asus tuf gaming a15 FA506II-0031A 4800H
- raspberry pi 3B+

##### Operating system:

- docker virtual environment from [NTURacingTeam/docker](https://github.com/NTURacingTeam/docker) with image `ros_matlab`, `ros_rpi` based on ubuntu20.04

##### Compiler(intepreter) version:

- python 3.8.10

---
## Testing result of 1.0:

### Launching

Launching by

```shell=
roslaunch nturt_bag_recorder nturt_bag_recorder.launch
```

launched and bag recorded successfully.


## Todos in 1.0:
