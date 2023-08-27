#!/usr/bin/python3

# python import
import datetime
import os

# ros2 import
import rosbag2_py

class BagRecorder:
    def __init__(self):

        # prevent bag file name conflict
        bag_uri = os.path.join(os.path.expanduser("~"), ".ros", "bags", datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
        if os.path.exists(bag_uri):
            surfix_count = 1
            while os.path.exists(bag_uri + "_" + str(surfix_count)):
                surfix_count += 1

            bag_uri = bag_uri + "_" + str(surfix_count)

        self.storage_options = rosbag2_py.StorageOptions(
            uri=bag_uri,
            storage_id="sqlite3",
            max_bagfile_duration=10,
            storage_preset_profile="resilient"
        )

        self.record_options = rosbag2_py.RecordOptions()
        self.record_options.topics = ["/from_can_bus", "/fix", "/vel", "/system_stats", "/rosout"]

        self.recorder = rosbag2_py.Recorder()
    
    def run(self):
        self.recorder.record(self.storage_options, self.record_options)

if __name__ == "__main__":
    bag_recorder = BagRecorder()

    try:
        bag_recorder.run()
    except KeyboardInterrupt:
        pass
