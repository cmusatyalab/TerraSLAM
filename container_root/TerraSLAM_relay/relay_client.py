import socket
import struct
import cv2
import os
import time
import glob
import argparse
import json
from pathlib import Path
import numpy as np
from pymap3d import enu2geodetic

# ------ Transform SLAM coordinates to LLA ------
class SLAM2GPS:
    def __init__(self,jpath):
        cfg=json.loads(Path(jpath).read_text())
        self.s,cfg_r,cfg_t = cfg["scale"], np.array(cfg["rotation"]), np.array(cfg["translation"])
        self.R,self.t = cfg_r, cfg_t.reshape(3,1)
        self.lat0,self.lon0,self.alt0 = cfg["lat0"],cfg["lon0"],cfg["alt0"]
    def slam_to_lla(self,xyz):
        xyz=np.asarray(xyz).reshape(3,-1)
        enu=(self.s*self.R@xyz+self.t).T
        return enu_to_lla(enu[:,0],enu[:,1],enu[:,2],
                          self.lon0,self.lat0,self.alt0)

def enu_to_lla(E, N, U, lon0, lat0, alt0):
    lat, lon, alt = enu2geodetic(E, N, U, lat0, lon0, alt0)
    return np.c_[lat, lon, alt]
# ------End of Transform SLAM coordinates to LLA -----------


class ImageClient:
    def __init__(self, server_ip='localhost', server_port=43322, transform_json='transform.json'):
        self.server_ip = server_ip
        self.server_port = server_port
        self.client_socket = None
        self.slam2gps = SLAM2GPS(transform_json)

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
            # print(f"Sent image: {image_path}")

            # Receive pose data (3 doubles = 24 bytes)
            pose_data = self.client_socket.recv(24)
            if len(pose_data) == 24:
                x, y, z = struct.unpack('3d', pose_data)
                # Check if all values are 0 (initializing status)
                if abs(x) < 1e-10 and abs(y) < 1e-10 and abs(z) < 1e-10:
                    print("System status: Initializing")
                else:
                    # Transform SLAM coordinates to LLA
                    lat, lon, alt = self.slam2gps.slam_to_lla([x, y, z])[0]
                    print(f"{lat},{lon},{alt}")
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
    parser.add_argument('--server', '-s', default='localhost', help='Server IP address (default: localhost)')
    parser.add_argument('--transform', '-t', default='transform.json', help='Path to transform.json file (default: transform.json)')
    args = parser.parse_args()

    # Check if folder exists
    if not os.path.exists(args.image_folder):
        print(f"Folder not found: {args.image_folder}")
        return

    # Check if transform file exists
    if not os.path.exists(args.transform):
        print(f"Transform file not found: {args.transform}")
        return

    # Create client
    client = ImageClient(server_ip=args.server, transform_json=args.transform)
    
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
            time.sleep(0.1)  # Small delay between images

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        client.close()

if __name__ == '__main__':
    main() 