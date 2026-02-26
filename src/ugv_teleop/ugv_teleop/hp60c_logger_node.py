#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, os, struct, zlib, numpy as np

class HP60CLogger(Node):
    def __init__(self):
        super().__init__('hp60c_logger')
        self.bridge = CvBridge()

        # dataset dir
        self.root = os.path.expanduser('~/datasets/hp60c_rgbd')
        os.makedirs(self.root, exist_ok=True)

        # open log file
        self.log_path = os.path.join(self.root, "hp60c_openni_2.log")
        self.log_file = open(self.log_path, "wb")
        self.log_file.write(struct.pack("<i", 0))  # placeholder numFrames
        self.written = 0

        # subs
        self.rgb_sub = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.rgb_callback,
            10)

        self.depth_sub = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            self.depth_callback,
            10)

        self.latest_rgb = None
        self.latest_depth = None

    def rgb_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest_rgb = (ts, cv_img)
        self.try_write_frame()

    def depth_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        depth_mm = cv_img.astype('uint16')

        # Median filtering (salt-and-pepper removal)
        depth_mm = cv2.medianBlur(depth_mm, 5)
        
        # Apply bilateral filter (edge-preserving, smoother than median)
        #depth_mm = cv2.bilateralFilter(depth_mm, d=5, sigmaColor=75, sigmaSpace=75)

        # Clip to range [0.3m, 4m]
        depth_mm[(depth_mm < 300) | (depth_mm > 4000)] = 0

        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest_depth = (ts, depth_mm)
        self.try_write_frame()



    def try_write_frame(self):
        if self.latest_rgb and self.latest_depth:
            tr, rgb = self.latest_rgb
            td, depth = self.latest_depth

            # crude sync
            if abs(tr - td) < 0.05:
                self.write_openni_frame(tr, rgb, depth)
                self.latest_rgb = None
                self.latest_depth = None

    def write_openni_frame(self, ts, rgb, depth):
        # encode RGB as JPEG
        ok, jpg = cv2.imencode(".jpg", rgb)
        if not ok:
            return
        jpg_bytes = jpg.tobytes()
        imageSize = len(jpg_bytes)

        # depth → uint16 mm
        # HP60C already provides depth in millimeters (uint16)
        depth_mm = depth.astype(np.uint16)
        depth_bytes = depth_mm.tobytes()
        depth_comp = zlib.compress(depth_bytes, level=1)
        depthSize = len(depth_comp)

        # timestamp in microseconds
        ts_us = int(ts * 1e6)

        # write frame
        self.log_file.write(struct.pack("<q", ts_us))     # int64
        self.log_file.write(struct.pack("<i", depthSize)) # int32
        self.log_file.write(struct.pack("<i", imageSize)) # int32
        self.log_file.write(depth_comp)
        self.log_file.write(jpg_bytes)

        self.written += 1
        if self.written % 50 == 0:
            self.get_logger().info(f"Written {self.written} frames")

    def finalize(self):
        # go back and write total frame count
        self.log_file.seek(0)
        self.log_file.write(struct.pack("<i", self.written))
        self.log_file.close()
        self.get_logger().info(f"Finalized log with {self.written} frames at {self.log_path}")

def main(args=None):
    rclpy.init(args=args)
    node = HP60CLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

