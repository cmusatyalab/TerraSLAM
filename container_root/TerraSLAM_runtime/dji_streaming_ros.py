import cv2
import os
from datetime import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# RTMP 流地址
rtmp_url = "rtmp://localhost:1935/live/stream"

# 创建保存帧的主文件夹
main_folder = "../Database"
if not os.path.exists(main_folder):
    os.makedirs(main_folder)

# 创建以时间戳命名的子文件夹
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
save_folder = os.path.join(main_folder, f"dji_{timestamp}")
os.makedirs(save_folder)

# 打开 RTMP 流
cap = cv2.VideoCapture(rtmp_url)

# 获取并打印视频流的属性
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

print(f"视频分辨率: {frame_width}x{frame_height}")
print(f"帧率: {fps}")

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
    frames_per_second = 2  # 每秒保存的帧数
    save_interval = max(1, int(fps / frames_per_second))  # 计算保存间隔

    while True:
        # 读取一帧
        ret, frame = cap.read()
        if not ret:
            print("无法接收帧")
            break
        
        frame_count += 1
        if frame_count == 1:
            # 打印第一帧的实际尺寸
            print(f"第一帧的实际尺寸: {frame.shape[1]}x{frame.shape[0]}")

        # 每 save_interval 帧保存一次
        if frame_count % save_interval == 0:
            saved_count += 1
            frame_filename = os.path.join(save_folder, f"{saved_count:06d}.jpg")
            cv2.imwrite(frame_filename, frame)

        # 发布图像到ROS2话题
        image_publisher.publish_image(frame)

        # 显示帧
        cv2.imshow('DJI Stream', frame)

        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

    print(f"总共接收 {frame_count} 帧，保存了 {saved_count} 帧")
    print(f"帧已保存到文件夹: {save_folder}")

if __name__ == '__main__':
    main()
