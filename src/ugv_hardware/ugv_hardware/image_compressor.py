#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')

        # Tunable params
        self.declare_parameter('jpeg_quality', 40)
        self.declare_parameter('resize_width', 320)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('depth_max_mm', 4000)  # clip depth for better contrast (4m default)

        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.resize_width = self.get_parameter('resize_width').value
        self.min_interval = 1.0 / self.get_parameter('publish_rate_hz').value
        self.depth_max_mm = self.get_parameter('depth_max_mm').value

        self.bridge = CvBridge()
        self.last_color_pub_time = 0.0
        self.last_depth_pub_time = 0.0

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        # Color
        self.color_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, qos)
        self.color_pub = self.create_publisher(
            CompressedImage, '/camera/color/custom/image_raw/compressed', qos)

        # Depth (colorized preview, NOT for algorithmic use)
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, qos)
        self.depth_pub = self.create_publisher(
            CompressedImage, '/camera/depth/custom/image_raw/compressed', qos)

        self.get_logger().info(
            f'Compressor up: quality={self.jpeg_quality}, width={self.resize_width}, '
            f'rate={1.0/self.min_interval}Hz, depth_max={self.depth_max_mm}mm'
        )

    def _throttle_ok(self, last_time):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - last_time < self.min_interval:
            return now, False
        return now, True

    def _resize(self, img):
        h, w = img.shape[:2]
        if w > self.resize_width:
            scale = self.resize_width / w
            new_size = (self.resize_width, int(h * scale))
            img = cv2.resize(img, new_size, interpolation=cv2.INTER_AREA)
        return img

    def _encode_and_publish(self, cv_img, header, publisher):
        params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        success, encoded = cv2.imencode('.jpg', cv_img, params)
        if not success:
            self.get_logger().warn('JPEG encode failed')
            return
        out_msg = CompressedImage()
        out_msg.header = header
        out_msg.format = 'jpeg'
        out_msg.data = encoded.tobytes()
        publisher.publish(out_msg)

    def color_callback(self, msg: Image):
        now, ok = self._throttle_ok(self.last_color_pub_time)
        if not ok:
            return
        self.last_color_pub_time = now

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'color cv_bridge failed: {e}')
            return

        cv_img = self._resize(cv_img)
        self._encode_and_publish(cv_img, msg.header, self.color_pub)

    def depth_callback(self, msg: Image):
        now, ok = self._throttle_ok(self.last_depth_pub_time)
        if not ok:
            return
        self.last_depth_pub_time = now

        try:
            # depth is typically 16UC1, values in millimeters
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().warn(f'depth cv_bridge failed: {e}')
            return

        # Clip and normalize to 0-255, then apply colormap (same idea as RealSense Viewer)
        depth_clipped = np.clip(depth_raw, 0, self.depth_max_mm)
        depth_8bit = (depth_clipped / self.depth_max_mm * 255).astype(np.uint8)
        depth_colorized = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)

        depth_colorized = self._resize(depth_colorized)
        self._encode_and_publish(depth_colorized, msg.header, self.depth_pub)


def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
