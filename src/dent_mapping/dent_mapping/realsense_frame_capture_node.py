#!/usr/bin/env python3
"""
realsense_frame_capture_node.py

ROS2 node that syncs RealSense color + aligned-depth + camera_info, and on
request (service call or auto-capture timer) writes a frame pair to disk in
the exact layout dent_reconstruction_v2.py already expects:

    <output_dir>/rgb/000000.png, 000001.png, ...
    <output_dir>/depth/000000.png, 000001.png, ...   (16-bit mm PNG)
    <output_dir>/intrinsics.json                      (written once)

This is step 1 toward real-time: it decouples "getting good synced frames
off the camera" from "running the matching/registration pipeline", so you
can validate capture quality independently before wiring the two together.

Requires: realsense2_camera launched separately, e.g.
    ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

Topics assume the default realsense-ros namespace; adjust the node
parameters below if yours differs (check `ros2 topic list`).

Add to your package's setup.py entry_points:
    'realsense_capture_node = <your_pkg>.realsense_frame_capture_node:main'
"""

import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from cv_bridge import CvBridge


class RealsenseFrameCaptureNode(Node):
    def __init__(self):
        super().__init__("realsense_frame_capture_node")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("output_dir", str(Path.home() / "data_3"))
        self.declare_parameter("auto_capture", False)
        self.declare_parameter("capture_rate_hz", 1.0)
        self.declare_parameter("show_preview", True)
        self.declare_parameter("sync_slop_sec", 0.03)

        self.output_dir = Path(self.get_parameter("output_dir").value)
        (self.output_dir / "rgb").mkdir(parents=True, exist_ok=True)
        (self.output_dir / "depth").mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.frame_count = 0
        self.latest_color = None
        self.latest_depth = None
        self.intrinsics_saved = False

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("camera_info_topic").value

        self.color_sub = message_filters.Subscriber(self, Image, color_topic, qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10,
            slop=self.get_parameter("sync_slop_sec").value)
        self.sync.registerCallback(self.synced_callback)

        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)

        self.capture_srv = self.create_service(Trigger, "capture_frame", self.capture_frame_service)

        if self.get_parameter("auto_capture").value:
            period = 1.0 / max(self.get_parameter("capture_rate_hz").value, 1e-3)
            self.timer = self.create_timer(period, self.auto_capture_callback)

        self.get_logger().info(
            f"Ready. Frames -> {self.output_dir}. "
            f"Call `ros2 service call /capture_frame std_srvs/srv/Trigger` to grab a frame, "
            f"or set auto_capture:=true for continuous capture."
        )

    def camera_info_callback(self, msg: CameraInfo):
        if self.intrinsics_saved:
            return
        fx, fy = msg.k[0], msg.k[4]
        cx, cy = msg.k[2], msg.k[5]
        intr_path = self.output_dir / "intrinsics.json"
        with open(intr_path, "w") as f:
            json.dump({"fx": fx, "fy": fy, "cx": cx, "cy": cy,
                       "width": msg.width, "height": msg.height}, f, indent=2)
        self.intrinsics_saved = True
        self.get_logger().info(f"Saved intrinsics to {intr_path}: fx={fx:.3f} fy={fy:.3f} cx={cx:.3f} cy={cy:.3f}")

    def synced_callback(self, color_msg: Image, depth_msg: Image):
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        # aligned_depth_to_color is typically 16UC1 in millimeters
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        self.latest_color = color
        self.latest_depth = depth

        if self.get_parameter("show_preview").value:
            depth_vis = cv2.convertScaleAbs(depth, alpha=0.03)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            preview = np.hstack([color, depth_vis])
            cv2.imshow("RealSense preview (color | depth)", preview)
            cv2.waitKey(1)

    def auto_capture_callback(self):
        self._save_current_frame()

    def capture_frame_service(self, request, response):
        ok, msg = self._save_current_frame()
        response.success = ok
        response.message = msg
        return response

    def _save_current_frame(self):
        if self.latest_color is None or self.latest_depth is None:
            return False, "No synced frame available yet."

        idx_str = f"{self.frame_count:06d}"
        rgb_path = self.output_dir / "rgb" / f"{idx_str}.png"
        depth_path = self.output_dir / "depth" / f"{idx_str}.png"

        cv2.imwrite(str(rgb_path), self.latest_color)
        # Preserve raw 16-bit mm depth so it matches what
        # dent_reconstruction_v2.py expects (it auto-detects mm vs metric
        # via the max-value heuristic).
        cv2.imwrite(str(depth_path), self.latest_depth.astype(np.uint16))

        self.frame_count += 1
        msg = f"Saved frame {idx_str} -> {rgb_path.name}, {depth_path.name}"
        self.get_logger().info(msg)
        return True, msg


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseFrameCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
