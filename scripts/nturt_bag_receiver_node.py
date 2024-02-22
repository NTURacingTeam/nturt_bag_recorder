#!/usr/bin/python3

import os
from typing import List
import rclpy
from rclpy.node import Node

BAGS_PATH = "/home/docker/ws/src/raw_data"
# the path on the remote host

class BagReceiver(Node):
    def __init__(self):
        super().__init__("nturt_bag_receiver_node")
        self.timer = self.create_timer(10, self.timer_callback)
        self.decoded_bags = [d.replace("decoded_merged_", "") for d in os.listdir(BAGS_PATH) if "decoded_merged" in d]        
        os.chdir(BAGS_PATH)

    def timer_callback(self):
        self.get_logger().info("Start Checking")
        current_bags = [d for d in os.listdir(BAGS_PATH) if "merged" not in d and d not in self.decoded_bags]
        for current_bag in current_bags:
            os.system(f"ros2 run nturt_bag_recorder merge_decode_bag.sh -md {BAGS_PATH}/{current_bag}")
            self.decoded_bags.append(current_bag)
            self.get_logger().info(f"{current_bag} bag received and start decoding.")

if __name__ == "__main__":
    rclpy.init()
    bag_receiver = BagReceiver()
    rclpy.spin(bag_receiver)
    bag_receiver.destroy_node()
    rclpy.shutdown()