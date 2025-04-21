import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import socket
import struct
import cv2
import numpy as np
import threading
import time

class ImageRelay(Node):
    def __init__(self):
        super().__init__('image_relay')
        
        # Initialize ROS2 publisher and subscriber
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/camera_pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        
        # Initialize camera pose
        self.current_pose = None
        self.pose_lock = threading.Lock()
        
        # Initialize TCP server
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', 43322))  # Listen on all interfaces
        self.server_socket.listen(1)
        self.get_logger().info('TCP server started on port 43322')
        
        # Start TCP server in a separate thread
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    def pose_callback(self, msg):
        # Update current pose with thread safety
        with self.pose_lock:
            self.current_pose = msg
            self.get_logger().info('Received new pose')

    def send_pose(self, client_socket):
        with self.pose_lock:
            if self.current_pose is not None:
                try:
                    # Pack pose data: x, y, z with high precision
                    pose_data = struct.pack('3d',  # Using double precision (d) instead of float (f)
                        self.current_pose.position.x,
                        self.current_pose.position.y,
                        self.current_pose.position.z
                    )
                    client_socket.sendall(pose_data)
                    self.get_logger().info('Sent pose data to client')
                except Exception as e:
                    self.get_logger().error(f'Failed to send pose: {e}')
            else:
                # Send initializing status (all zeros)
                try:
                    # Pack 3 zeros with double precision
                    pose_data = struct.pack('3d', 0.0, 0.0, 0.0)
                    client_socket.sendall(pose_data)
                    self.get_logger().info('Sent initializing status to client')
                except Exception as e:
                    self.get_logger().error(f'Failed to send initializing status: {e}')

    def handle_client(self, client_socket):
        try:
            while True:
                # Receive image size
                size_data = client_socket.recv(4)
                if not size_data:
                    self.get_logger().info('Client disconnected')
                    break
                
                image_size = struct.unpack('!I', size_data)[0]
                self.get_logger().info(f'Expecting image of size: {image_size} bytes')
                
                # Receive image data
                received_data = b''
                while len(received_data) < image_size:
                    chunk = client_socket.recv(min(4096, image_size - len(received_data)))
                    if not chunk:
                        self.get_logger().error('Connection broken while receiving image')
                        break
                    received_data += chunk
                
                if len(received_data) != image_size:
                    self.get_logger().error('Incomplete image received')
                    continue
                
                # Convert received data to image
                try:
                    # Decode image
                    nparr = np.frombuffer(received_data, np.uint8)
                    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    if image is None:
                        self.get_logger().error('Failed to decode image')
                        continue
                    
                    self.get_logger().info(f'Successfully decoded image of size {image.shape}')
                    
                    # Publish image to ROS2
                    msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                    self.publisher_.publish(msg)
                    self.get_logger().info('Published image to ROS2')
                    
                    # Send current pose back to client
                    self.send_pose(client_socket)
                    
                except Exception as e:
                    self.get_logger().error(f'Error processing image: {e}')
                
        except Exception as e:
            self.get_logger().error(f'Client connection error: {e}')
        finally:
            client_socket.close()

    def run_server(self):
        try:
            while True:
                client_socket, address = self.server_socket.accept()
                self.get_logger().info(f'New connection from {address}')
                client_thread = threading.Thread(target=self.handle_client, args=(client_socket,))
                client_thread.daemon = True
                client_thread.start()
        except Exception as e:
            self.get_logger().error(f'Server error: {e}')
        finally:
            self.server_socket.close()

def main(args=None):
    rclpy.init(args=args)
    image_relay = ImageRelay()
    try:
        rclpy.spin(image_relay)
    except KeyboardInterrupt:
        image_relay.get_logger().info('Shutting down image relay.')
    finally:
        image_relay.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
