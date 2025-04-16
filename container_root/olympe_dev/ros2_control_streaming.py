#!/usr/bin/env python3

import csv
import math
import os
import queue
import shlex
import subprocess
import tempfile
import threading
import time
import cv2
import numpy as np
import datetime
import inputs
import logging

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingEvent import moveByEnd
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.video.renderer import PdrawRenderer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from multiprocessing import Value

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})


DRONE_IP = "192.168.42.1"
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")

class StreamingExample(Node):
    def __init__(self):
        super().__init__('drone_image_publisher')
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.drone = olympe.Drone(DRONE_IP)
        current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        file = tempfile.mkdtemp(prefix=f"olympe_streaming_{current_time}_", dir="./")
        self.output_dir = os.path.abspath(file)
        self.get_logger().info(f"Olympe streaming example output dir: {self.output_dir}")
        self.h264_frame_stats = []
        self.h264_stats_file = open(os.path.join(self.output_dir, "h264_stats.csv"), "w+")
        self.h264_stats_writer = csv.DictWriter(self.h264_stats_file, ["fps", "bitrate"])
        self.h264_stats_writer.writeheader()
        self.frame_queue = queue.Queue()
        self.processing_thread = threading.Thread(target=self.yuv_frame_processing)
        self.renderer = None
        self.is_publishing = Value('b', False)  # Shared boolean value
        self.publish_lock = threading.Lock()
        self.last_bgr_frame = None
        self.new_frame_available = threading.Event()
        self.drone_state = "landed"
        self.drone.subscribe(self.flying_state_changed)

    def flying_state_changed(self, event, sender):
        self.drone_state = event.args["state"]
        self.get_logger().info(f"Drone state changed to: {self.drone_state}")

    def start(self):
        assert self.drone.connect(retry=3)
        if DRONE_RTSP_PORT is not None:
            self.drone.streaming.server_addr = f"{DRONE_IP}:{DRONE_RTSP_PORT}"
        self.drone.streaming.set_output_files(metadata=os.path.join(self.output_dir, "streaming_metadata.json"))
        self.drone.streaming.set_callbacks(raw_cb=self.yuv_frame_cb, h264_cb=self.h264_frame_cb, start_cb=self.start_cb, end_cb=self.end_cb, flush_raw_cb=self.flush_cb)
        self.drone.streaming.start()
        self.renderer = PdrawRenderer(pdraw=self.drone.streaming)
        self.running = True
        self.processing_thread.start()
        threading.Thread(target=self.keyboard_listener).start()
        threading.Thread(target=self.image_publisher).start()

    def stop(self):
        self.running = False
        self.processing_thread.join()
        if self.renderer is not None:
            self.renderer.stop()
        assert self.drone.streaming.stop()
        assert self.drone.disconnect()
        self.h264_stats_file.close()

    def take_off(self):
        self.get_logger().info("Attempting to take off...")
        takeoff_command = self.drone(
            TakeOff()
            & FlyingStateChanged(state="hovering", _timeout=5)
            | FlyingStateChanged(state="flying", _timeout=5)
        )
        takeoff_result = takeoff_command.wait()
        if takeoff_result.success():
            self.get_logger().info("Take off successful.")
        else:
            self.get_logger().info(f"Take off failed.")
        self.get_logger().info(f"Current drone state: {self.drone_state}")
    
    def land(self):
        if self.drone_state not in ["hovering", "flying"]:
            self.get_logger().info(f"Drone is not flying. Current state: {self.drone_state}")
            return

        self.get_logger().info("Attempting to land...")
        landing_command = self.drone(
            Landing()
            >> FlyingStateChanged(state="landed", _timeout=10)
        )
        landing_result = landing_command.wait()
        if landing_result.success():
            self.get_logger().info("Landing successful.")
        else:
            self.get_logger().info("Landing failed.")
        self.get_logger().info(f"Current drone state: {self.drone_state}")

    def keyboard_listener(self):
        while self.running:
            events = inputs.get_key()
            for event in events:
                print(f"Event: {event.ev_type}, Code: {event.code}, State: {event.state}")
                if event.ev_type == 'Key' and event.state == 1:  # Only react to key presses
                    if event.code == 'KEY_T':
                        self.take_off()
                    elif event.code == 'KEY_L':
                        self.land()
                    elif event.code == 'KEY_W':
                        response = self.drone(moveBy(3, 0, 0, 0, _timeout=5)).wait()
                        print(f"Move forward response: {response}")
                    elif event.code == 'KEY_S':
                        response = self.drone(moveBy(-3, 0, 0, 0, _timeout=5)).wait()
                        print(f"Move backward response: {response}")
                    elif event.code == 'KEY_A':
                        response = self.drone(moveBy(0, -3, 0, 0, _timeout=5)).wait()
                        print(f"Move left response: {response}")
                    elif event.code == 'KEY_D':
                        response = self.drone(moveBy(0, 3, 0, 0, _timeout=5)).wait()
                        print(f"Move right response: {response}")
                    elif event.code == 'KEY_UP':
                        response = self.drone(moveBy(0, 0, -1, 0, _timeout=5)).wait()
                        print(f"Ascending response: {response}")
                    elif event.code == 'KEY_DOWN':
                        response = self.drone(moveBy(0, 0, 1, 0, _timeout=5)).wait()
                        print(f"Descending response: {response}")
                    elif event.code == 'KEY_Q':
                        response = self.drone(moveBy(0, 0, 0, -math.pi/8, _timeout=5)).wait()
                        print(f"Rotate left response: {response}")
                    elif event.code == 'KEY_E':
                        response = self.drone(moveBy(0, 0, 0, math.pi/8, _timeout=5)).wait()
                        print(f"Rotate right response: {response}")
                    elif event.code == 'KEY_P':
                        with self.is_publishing.get_lock():
                            self.is_publishing.value = not self.is_publishing.value
                        status = "started" if self.is_publishing.value else "stopped"
                        print(f"Image publishing {status}")

                    self.drone.subscribe(
                        lambda event, sender: print(f"Move ended with status: {event.args['error']}"),
                        moveByEnd()
                    )

    def yuv_frame_cb(self, yuv_frame):
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    def yuv_frame_processing(self):
        while self.running:
            try:
                yuv_frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            info = yuv_frame.info()
            yuv_array = np.frombuffer(yuv_frame.as_ndarray(), dtype=np.uint8)
            height, width = info["raw"]["frame"]["info"]["height"], info["raw"]["frame"]["info"]["width"]
            yuv_array = yuv_array.reshape((height * 3 // 2, width))
            bgr_frame = cv2.cvtColor(yuv_array, cv2.COLOR_YUV2BGR_I420)
            
            # Store the latest frame for publishing
            with self.publish_lock:
                self.last_bgr_frame = bgr_frame.copy()
            
            # Save the frame as an image file
            timestamp = int(time.time() * 1000)
            image_filename = os.path.join(self.output_dir, f"{timestamp}.jpg")
            cv2.imwrite(image_filename, bgr_frame)
            
            yuv_frame.unref()
            
            # Signal that a new frame is available
            self.new_frame_available.set()

    def flush_cb(self, stream):
        if stream["vdef_format"] != olympe.VDEF_I420:
            return True
        while not self.frame_queue.empty():
            self.frame_queue.get_nowait().unref()
        return True

    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def h264_frame_cb(self, h264_frame):
        frame_pointer, frame_size = h264_frame.as_ctypes_pointer()
        info = h264_frame.info()
        frame_ts = info["ntp_raw_timestamp"]
        if not bool(info["is_sync"]):
            while len(self.h264_frame_stats) > 0:
                start_ts, _ = self.h264_frame_stats[0]
                if (start_ts + 1e6) < frame_ts:
                    self.h264_frame_stats.pop(0)
                else:
                    break
            self.h264_frame_stats.append((frame_ts, frame_size))
            h264_fps = len(self.h264_frame_stats)
            h264_bitrate = 8 * sum(map(lambda t: t[1], self.h264_frame_stats))
            self.h264_stats_writer.writerow({"fps": h264_fps, "bitrate": h264_bitrate})

    def image_publisher(self):
        while self.running:
            # Wait for a new frame to be available
            self.new_frame_available.wait()
            self.new_frame_available.clear()
            
            with self.is_publishing.get_lock():
                if self.is_publishing.value:
                    with self.publish_lock:
                        if self.last_bgr_frame is not None:
                            ros_image = self.bridge.cv2_to_imgmsg(self.last_bgr_frame, encoding="bgr8")
                            self.image_pub.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    streaming_example = StreamingExample()
    streaming_example.start()
    # streaming_example.take_off()
    try:
        rclpy.spin(streaming_example)
    except KeyboardInterrupt:
        streaming_example.land()
    finally:
        streaming_example.stop()
        streaming_example.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

