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
import json
from pathlib import Path
from pymap3d import enu2geodetic
from transform_utils import transform_point, read_transform_matrix

# ------ Transform SLAM coordinates to LLA ------
class SLAM2GPS:
    def __init__(self, jpath):
        cfg = json.loads(Path(jpath).read_text())
        self.s, cfg_r, cfg_t = cfg["scale"], np.array(cfg["rotation"]), np.array(cfg["translation"])
        self.R, self.t = cfg_r, cfg_t.reshape(3,1)
        self.lat0, self.lon0, self.alt0 = cfg["lat0"], cfg["lon0"], cfg["alt0"]
    
    def slam_to_lla(self, xyz):
        xyz = np.asarray(xyz).reshape(3,-1)
        enu = (self.s*self.R@xyz+self.t).T
        return self.enu_to_lla(enu[:,0], enu[:,1], enu[:,2])
    
    def enu_to_lla(self, E, N, U):
        lat, lon, alt = enu2geodetic(E, N, U, self.lat0, self.lon0, self.alt0)
        return np.c_[lat, lon, alt]

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
        
        # Initialize GPS conversion
        self.transform_json = "tranform_0602_merge.json"  # Update this path to your transform json file
        try:
            self.slam2gps = SLAM2GPS(self.transform_json)
            self.base_transform_matrix = read_transform_matrix("slam_transform_matrix.txt")
            self.get_logger().info('GPS conversion initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPS conversion: {e}')
            self.slam2gps = None
            self.base_transform_matrix = None
        
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
                    # Extract pose data
                    x = self.current_pose.position.x
                    y = self.current_pose.position.y
                    z = self.current_pose.position.z
                    
                    # Check for special tracking status values
                    # Use exact equality for primitive types
                    if (x == -3.0 and y == -3.0 and z == -3.0):
                        # Tracking lost status
                        pose_data = struct.pack('3d', -3.0, -3.0, -3.0)
                        self.get_logger().info('Sent tracking lost status to client')
                    elif (x == -1.0 and y == -1.0 and z == -1.0):
                        # Not initialized status
                        pose_data = struct.pack('3d', -1.0, -1.0, -1.0)
                        self.get_logger().info('Sent not initialized status to client')
                    elif (x == 0.0 and y == 0.0 and z == 0.0):
                        # No images yet status
                        pose_data = struct.pack('3d', 0.0, 0.0, 0.0)
                        self.get_logger().info('Sent no images status to client')
                    # Convert to GPS if possible for regular tracking
                    elif self.slam2gps is not None and self.base_transform_matrix is not None:
                        try:
                            # Transform point using the base matrix
                            x_t, y_t, z_t = transform_point(self.base_transform_matrix, [x, y, z])
                            # Convert to GPS coordinates
                            lat, lon, alt = self.slam2gps.slam_to_lla([x_t, y_t, z_t])[0]
                            
                            # Pack GPS data: lat, lon, alt with high precision
                            pose_data = struct.pack('3d', lat, lon, alt)
                            self.get_logger().info(f'Sent GPS data to client: {lat}, {lon}, {alt}')
                        except Exception as e:
                            self.get_logger().error(f'Failed to convert to GPS: {e}')
                            # Fallback to raw pose data
                            pose_data = struct.pack('3d', x, y, z)
                            self.get_logger().info('Sent raw pose data to client')
                    else:
                        # No GPS conversion, send raw pose data
                        pose_data = struct.pack('3d', x, y, z)
                        self.get_logger().info('Sent raw pose data to client')
                        
                    client_socket.sendall(pose_data)
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
