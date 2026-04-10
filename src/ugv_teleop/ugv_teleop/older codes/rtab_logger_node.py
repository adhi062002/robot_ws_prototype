#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer
from collections import deque

class DualRatePublisher(Node):
    def __init__(self):
        super().__init__("dual_rate_publisher")

        # --------------------------
        # BUFFERS
        # --------------------------
        self.imu_buffer  = deque(maxlen=5000)      # (ts, imu_msg)
        self.rgbd_buffer = deque(maxlen=1000)      # (ts, rgb_msg, depth_msg)

        # thresholds = 90% full
        self.imu_start_threshold  = int(0.9 * 5000)   
        self.rgbd_start_threshold = int(0.9 * 1000)   

        self.imu_ready  = False
        self.rgbd_ready = False

        # --------------------------
        # INPUT SUBSCRIBERS
        # --------------------------
        self.create_subscription(Imu, "/imu/data_raw", self.imu_callback, 200)

        self.rgb_sub   = Subscriber(self, Image, "/ascamera_hp60c/camera_publisher/rgb0/image")
        self.depth_sub = Subscriber(self, Image, "/ascamera_hp60c/camera_publisher/depth0/image_raw")

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=50,
            slop=0.01
        )
        self.sync.registerCallback(self.rgb_depth_callback)

        # --------------------------
        # OUTPUT PUBLISHERS
        # --------------------------
        self.pub_imu   = self.create_publisher(Imu, "/out/imu", 10)
        self.pub_rgb   = self.create_publisher(Image, "/out/rgb", 10)
        self.pub_depth = self.create_publisher(Image, "/out/depth", 10)

        # --------------------------
        # TIMERS FOR CONTROLLED RATES
        # --------------------------
        self.imu_rate = 84.0
        self.rgbd_rate = 30.0

        self.imu_timer  = self.create_timer(1.0 / self.imu_rate, self.publish_imu)
        self.rgbd_timer = self.create_timer(1.0 / self.rgbd_rate, self.publish_rgbd)

        # Current publishing positions
        self.next_imu_index  = 0
        self.next_rgbd_index = 0

        self.get_logger().info("Waiting for buffers to fill before publishing...")

    # ------------------------------------------------------
    # IMU STORAGE
    # ------------------------------------------------------
    def imu_callback(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu_buffer.append((ts, msg))

        if not self.imu_ready and len(self.imu_buffer) >= self.imu_start_threshold:
            self.imu_ready = True
            self.get_logger().info(f"IMU buffer filled ({len(self.imu_buffer)}/{self.imu_buffer.maxlen}), publishing started.")

    # ------------------------------------------------------
    # RGBD STORAGE
    # ------------------------------------------------------
    def rgb_depth_callback(self, rgb_msg, depth_msg):
        ts = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        self.rgbd_buffer.append((ts, rgb_msg, depth_msg))

        if not self.rgbd_ready and len(self.rgbd_buffer) >= self.rgbd_start_threshold:
            self.rgbd_ready = True
            self.get_logger().info(f"RGBD buffer filled ({len(self.rgbd_buffer)}/{self.rgbd_buffer.maxlen}), publishing started.")

    # ------------------------------------------------------
    # PUBLISH IMU @ 84 Hz (ONLY WHEN READY)
    # ------------------------------------------------------
    def publish_imu(self):
        if not self.imu_ready & self.rgbd_ready:
            return

        if self.next_imu_index < len(self.imu_buffer):
            _, imu_msg = self.imu_buffer[self.next_imu_index]
            self.pub_imu.publish(imu_msg)
            self.next_imu_index += 1

    # ------------------------------------------------------
    # PUBLISH RGBD @ 30 Hz (ONLY WHEN READY)
    # ------------------------------------------------------
    def publish_rgbd(self):
        if not self.rgbd_ready & self.imu_ready:
            return

        if self.next_rgbd_index < len(self.rgbd_buffer):
            _, rgb_msg, depth_msg = self.rgbd_buffer[self.next_rgbd_index]

            # keep timestamps unchanged
            self.pub_rgb.publish(rgb_msg)
            self.pub_depth.publish(depth_msg)

            self.next_rgbd_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = DualRatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer
from collections import deque

class BufferedSyncPublisher(Node):
    def __init__(self):
        super().__init__("buffered_sync_publisher")

        # ---------- PARAMETERS ----------
        self.rgbd_buffer_size = 990    # buffer almost full → 90% of 100
        self.imu_buffer_size = 3900   # buffer almost full → 400% of 500
        self.start_rgbd_publishing = False
        self.start_imu_publishing = False

        # ---------- BUFFERS ----------
        self.rgbd_buffer = deque(maxlen=1000)   # each element = (rgb_msg, depth_msg)
        self.imu_buffer = deque(maxlen=4000)    # imu messages only

        # ---------- SUBSCRIBERS ----------
        self.create_subscription(Imu, "/imu/data_raw", self.imu_callback, 200)

        self.rgb_sub   = Subscriber(self, Image, "/ascamera_hp60c/camera_publisher/rgb0/image")
        self.depth_sub = Subscriber(self, Image, "/ascamera_hp60c/camera_publisher/depth0/image_raw")

        # Sync RGB + Depth within tight time (0.01 sec)
        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=50,
            slop=0.01
        )
        self.ts.registerCallback(self.rgbd_callback)

        # ---------- PUBLISHERS ----------
        self.rgb_pub = self.create_publisher(Image, "/out/rgb", 10)
        self.depth_pub = self.create_publisher(Image, "/out/depth", 10)
        self.imu_pub = self.create_publisher(Imu, "/out/imu", 50)

        # ---------- TIMERS ----------
        self.imu_timer = self.create_timer(1.0 / 84.0, self.publish_imu)
        self.rgbd_timer = self.create_timer(1.0 / 30.0, self.publish_rgbd)

        self.get_logger().info("Buffered sync publisher started.")

    # ------------------------------------------------------------
    # BUFFER CALLBACKS
    # ------------------------------------------------------------

    def rgbd_callback(self, rgb_msg, depth_msg):
        """Store synced RGB + depth with original timestamps."""
        self.rgbd_buffer.append((rgb_msg, depth_msg))

        # Start publishing if buffer nearly full
        if not self.start_rgbd_publishing and len(self.rgbd_buffer) >= self.rgbd_buffer_size:
            self.start_rgbd_publishing = True
            self.get_logger().info("RGBD buffer almost full → Starting publishing.")

    def imu_callback(self, imu_msg):
        """Store IMU with original timestamp."""
        self.imu_buffer.append(imu_msg)

        if not self.start_imu_publishing and len(self.imu_buffer) >= self.imu_buffer_size:
            self.start_imu_publishing = True
            self.get_logger().info("IMU buffer almost full → Starting publishing.")

    # ------------------------------------------------------------
    # TIMER CALLBACKS
    # ------------------------------------------------------------

    def publish_rgbd(self):
        """Publish RGB + depth at 30 Hz using stored timestamps."""
        if not self.start_rgbd_publishing & self.start_imu_publishing:
            return
        if len(self.rgbd_buffer) == 0:
            return

        rgb_msg, depth_msg = self.rgbd_buffer.popleft()

        # DO NOT MODIFY TIMESTAMPS
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)

    def publish_imu(self):
        """Publish IMU at 84 Hz using stored timestamps."""
        if not self.start_imu_publishing & self.start_rgbd_publishing:
            return
        if len(self.imu_buffer) == 0:
            return

        imu_msg = self.imu_buffer.popleft()

        # DO NOT MODIFY TIMESTAMP
        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BufferedSyncPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
