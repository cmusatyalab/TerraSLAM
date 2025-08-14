import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import socket
import json
import sys

class PoseTCPClient(Node):
    def __init__(self, header_string="default"):
        super().__init__('pose_tcp_client')
        self.header_string = header_string
        self.subscription = self.create_subscription(
            Pose,
            '/camera_pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.pose_data = None
        self.tcp_host = '127.0.0.1'  # Replace with your Blender server IP
        self.tcp_port = 11223  # Replace with the appropriate port
        self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connect_to_server()
        self.timer = self.create_timer(1.0/30, self.send_pose_data)  # Timer to send data every second

    def connect_to_server(self):
        try:
            self.tcp_client.connect((self.tcp_host, self.tcp_port))
            self.get_logger().info(f'Connected to server at {self.tcp_host}:{self.tcp_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to server: {e}')

    def pose_callback(self, msg):
        self.pose_data = msg

    def send_pose_data(self):
        if self.pose_data:
            position = {
                'x': self.pose_data.position.x,
                'y': self.pose_data.position.y,
                'z': self.pose_data.position.z
            }
            quaternion = {
                'w': self.pose_data.orientation.w,
                'x': self.pose_data.orientation.x,
                'y': self.pose_data.orientation.y,
                'z': self.pose_data.orientation.z
            }
            pose_data = {
                'header': self.header_string,
                'position': position,
                'quaternion': quaternion
            }
            data = json.dumps(pose_data)
            try:
                self.tcp_client.sendall(data.encode('utf-8'))
                self.get_logger().info(f'Sent data: {data}')
            except Exception as e:
                self.get_logger().error(f'Failed to send data: {e}')
                self.tcp_client.close()
                self.connect_to_server()

def main(args=None):
    rclpy.init(args=args)
    
    # Get header string from command line arguments
    header_string = "default"
    if len(sys.argv) > 1:
        header_string = sys.argv[1]
    
    pose_tcp_client = PoseTCPClient(header_string)
    try:
        rclpy.spin(pose_tcp_client)
    except KeyboardInterrupt:
        pose_tcp_client.get_logger().info('Shutting down pose TCP client.')
    finally:
        pose_tcp_client.tcp_client.close()
        pose_tcp_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
