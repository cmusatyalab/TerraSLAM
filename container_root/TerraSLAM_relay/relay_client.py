import socket
import struct
import cv2
import os
import time
import glob
import argparse

class ImageClient:
    def __init__(self, server_ip, server_port=43322):
        self.server_ip = server_ip
        self.server_port = server_port
        self.client_socket = None
        # Server now handles GPS conversion and sends GPS coordinates directly
        self.latest_pose = None  # Store the latest pose data

    def connect(self):
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.server_ip, self.server_port))
            print(f"Connected to server at {self.server_ip}:{self.server_port}")
            return True
        except Exception as e:
            print(f"Failed to connect to server: {e}")
            return False

    def send_image(self, image_path):
        try:
            # Read image
            image = cv2.imread(image_path)
            if image is None:
                print(f"Failed to read image: {image_path}")
                return False

            # Encode image
            _, img_encoded = cv2.imencode('.jpg', image)
            img_bytes = img_encoded.tobytes()

            # Send image size
            size = len(img_bytes)
            self.client_socket.sendall(struct.pack('!I', size))

            # Send image data
            self.client_socket.sendall(img_bytes)

            # Receive pose data (3 doubles = 24 bytes)
            # Server now sends GPS coordinates directly: (lat, lon, alt) or status values
            pose_data = self.client_socket.recv(24)
            if len(pose_data) == 24:
                lat, lon, alt = struct.unpack('3d', pose_data)
                self.latest_pose = (lat, lon, alt)  # Store the latest GPS coordinates
                
                # Check for special status values as defined in relay.py
                if lat == -3.0 and lon == -3.0 and alt == -3.0:
                    print("System status: Tracking lost")
                elif lat == -1.0 and lon == -1.0 and alt == -1.0:
                    print("System status: Not initialized")
                elif lat == 0.0 and lon == 0.0 and alt == 0.0:
                    print("System status: Initializing / No images yet")
                else:
                    # Valid GPS coordinates received directly from server
                    print(f"GPS: {lat:.8f}, {lon:.8f}, {alt:.3f}m")
            else:
                print("Failed to receive complete pose data")
                return False

            return True

        except Exception as e:
            print(f"Error sending image {image_path}: {e}")
            return False

    def close(self):
        if self.client_socket:
            self.client_socket.close()
            print("Connection closed")

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Send images to server and receive poses')
    parser.add_argument('image_folder', help='Path to the folder containing images')
    parser.add_argument('--server', '-s', default='128.2.208.19', help='Server IP address (default: 128.2.208.19)')
    parser.add_argument('--port', '-p', default=43322, help='Server port (default: 43322)')
    args = parser.parse_args()

    # Check if folder exists
    if not os.path.exists(args.image_folder):
        print(f"Folder not found: {args.image_folder}")
        return

    # Create client
    client = ImageClient(server_ip=args.server, server_port=int(args.port))
    
    # Connect to server
    if not client.connect():
        return

    try:
        # Get all jpg files and sort them
        image_files = sorted(glob.glob(os.path.join(args.image_folder, '*.jpg')))
        if not image_files:
            print("No jpg files found in the folder")
            return

        print(f"Found {len(image_files)} images")
        
        # Send each image
        for image_path in image_files:
            if not client.send_image(image_path):
                break
            time.sleep(0.03)  # Small delay between images

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        client.close()

if __name__ == '__main__':
    main() 