import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import csv
import sys
import os

class PoseRecorder(Node):
    def __init__(self):
        super().__init__('pose_recorder')
        self.subscription = self.create_subscription(
            Pose,
            '/camera_pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Delete existing CSV file if it exists
        csv_file_path = 'camera_pose.csv'
        if os.path.exists(csv_file_path):
            os.remove(csv_file_path)
            self.get_logger().info('Deleted existing CSV file')
        
        # Initialize CSV file
        self.csv_file = open(csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz'])
        self.get_logger().info('Pose recorder initialized')

    def pose_callback(self, msg):
        # Write to CSV file
        self.csv_writer.writerow([
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])
        self.csv_file.flush()  # Ensure data is written to disk

    def __del__(self):
        # Close CSV file when object is destroyed
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
            self.get_logger().info('CSV file closed')

def main(args=None):
    rclpy.init(args=args)
    pose_recorder = PoseRecorder()
    try:
        rclpy.spin(pose_recorder)
    except KeyboardInterrupt:
        pose_recorder.get_logger().info('Shutting down pose recorder.')
    finally:
        pose_recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
