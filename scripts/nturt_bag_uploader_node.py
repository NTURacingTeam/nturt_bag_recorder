#!/usr/bin/python3

import paramiko
import os
from RPI.GPIO import GPIO
import rclpy
from rclpy.node import Node

HOST = "140.112.14.14"
USERNAME = "user"
PASSWORD = "nturacing"
PORT = 22
BAGS_PATH = "/home/docker/.ros/bags"
REMOTE_BAGS_PATH = "/home/user/Documents/docker/packages/ros2/raw_data"
BUTTON_PIN = 16
pressed = False


class BagUploader(Node):
    def __init__(self):
        super().__init__("nturt_bag_uploader_node")

        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh_client.connect(
            hostname=HOST,
            port=PORT,
            username=USERNAME,
            password=PASSWORD
        )
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.sftp = self.ssh_client.open_sftp()
        self.get_logger().info('bag uploader connection established successfully')

    def run(self):
        remote_exist_bags = self.sftp.listdir(REMOTE_BAGS_PATH)
        bags = [d for d in os.listdir(BAGS_PATH) if d not in remote_exist_bags and "merged" not in d]

        for bag in bags:
            bag_path = os.path.join(BAGS_PATH, bag)
            remote_bag_path = os.path.join(REMOTE_BAGS_PATH, bag)

            files = [f for f in os.listdir(bag_path)]
            self.sftp.mkdir(remote_bag_path)
            for file in files:
                self.sftp.put(os.path.join(bag_path, file), os.path.join(remote_bag_path, file))

            self.get_logger().info(f"{bag_path} bag uploaded to remote host.")

    def __del__(self):
        self.sftp.close()
        self.ssh_client.close()

if __name__ == "__main__":
    rclpy.init()
    bag_uploader = BagUploader()

    try:
        while True:
            if not GPIO.input(BUTTON_PIN):
                if not pressed:
                    bag_uploader.run()
                    pressed = True
            # button not pressed (or released)
            else:
                pressed = False
    except KeyboardInterrupt:
        pass
    finally:
        del bag_uploader