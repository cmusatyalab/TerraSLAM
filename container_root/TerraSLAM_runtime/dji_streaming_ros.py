import cv2
import os
from datetime import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# RTMP stream address
rtmp_url = "rtmp://localhost:1935/live/stream"

# Create main folder for saving frames
main_folder = "../Database"
if not os.path.exists(main_folder):
    os.makedirs(main_folder)

# Create subfolder named with timestamp
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
save_folder = os.path.join(main_folder, f"dji_{timestamp}")
os.makedirs(save_folder)

# Open RTMP stream
cap = cv2.VideoCapture(rtmp_url)

# Get and print video stream properties
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

print(f"Video resolution: {frame_width}x{frame_height}")
print(f"Frame rate: {fps}")

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

    def publish_image(self, frame):
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)

def main():
    # Initialize ROS2
    rclpy.init()
    image_publisher = ImagePublisher()

    frame_count = 0
    saved_count = 0
    frames_per_second = 2  # Number of frames to save per second
    save_interval = max(1, int(fps / frames_per_second))  # Calculate save interval

    while True:
        # Read a frame
        ret, frame = cap.read()
        if not ret:
            print("Unable to receive frame")
            break
        
        frame_count += 1
        if frame_count == 1:
            # Print actual size of first frame
            print(f"Actual size of first frame: {frame.shape[1]}x{frame.shape[0]}")

        # Save every save_interval frames
        if frame_count % save_interval == 0:
            saved_count += 1
            frame_filename = os.path.join(save_folder, f"{saved_count:06d}.jpg")
            cv2.imwrite(frame_filename, frame)

        # Publish image to ROS2 topic
        image_publisher.publish_image(frame)

        # Display frame
        cv2.imshow('DJI Stream', frame)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

    print(f"Total received {frame_count} frames, saved {saved_count} frames")
    print(f"Frames saved to folder: {save_folder}")

if __name__ == '__main__':
    main()
