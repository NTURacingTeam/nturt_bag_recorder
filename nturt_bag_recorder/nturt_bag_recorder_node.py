# python3 import
import os
import subprocess

# ros2 import
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory

class BagRecorder(rclpy.node.Node):
    def __init__(self):
        super().__init__('nturt_bag_record_node')

        # define default parameters
        nturt_bag_recorder_package = get_package_share_directory('nturt_bag_recorder')
        default_record_script = os.path.join(nturt_bag_recorder_package, 'scripts', 'nturt_bag_recorder.sh')
        default_record_folder = os.path.join(nturt_bag_recorder_package, 'bags')

        # declare parameters
        self.declare_parameter('record_script', default_record_script)
        self.declare_parameter('record_folder', default_record_folder)

        # get parameters
        self.record_script = self.get_parameter('record_script').get_parameter_value().string_value
        self.record_folder = self.get_parameter('record_folder').get_parameter_value().string_value

        # start recording
        # seems no need to shut the bag record node down because subprocess is doing tht for us when the program exits
        self.get_logger().info(f'Start recording to {self.record_folder} with recording script {self.record_script}.')
        command = "source " + self.record_script
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.record_folder, executable='/bin/bash')

def main(args=None):
    rclpy.init(args=args)

    bag_recorder_node = BagRecorder()

    rclpy.spin(bag_recorder_node)

    bag_recorder_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
