import os
import cv2
import rclpy
import sys
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self, image_dir):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.04, self.timer_callback)
        self.image_files = sorted(os.listdir(image_dir), key=lambda x: float(x.split('.')[0]))
        self.image_dir = image_dir
        self.index = 0

    def timer_callback(self):
        if self.index < len(self.image_files):
            img_path = os.path.join(self.image_dir, self.image_files[self.index])
            cv_image = cv2.imread(img_path)
            if cv_image is not None:
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                self.publisher.publish(ros_image)
                self.get_logger().info(f'Publishing {self.image_files[self.index]}')
            self.index += 1
        else:
            self.index = 0  # Restart or stop timer

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: python main.py <path_to_images>")
        return
    image_dir = sys.argv[1]
    image_publisher = ImagePublisher(image_dir)
    try:
        rclpy.spin(image_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        image_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
