#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, Imu


class TUMRGBDLogger(Node):
    def __init__(self,
                 root_dir="~/datasets/hp60c_rgbdi",
                 rgb_topic="/ascamera_hp60c/camera_publisher/rgb0/image",
                 depth_topic="/ascamera_hp60c/camera_publisher/depth0/image_raw",
                 imu_topic="/imu/data_raw",
                 slop_rgbd=0.04,         # tight RGB-D sync (30 ms)
                 slop_rgbd_imu=0.05):    # looser RGB-D-IMU sync (50 ms)

        super().__init__('tum_rgbd_logger')
        self.bridge = CvBridge()

        # Directories
        self.root = os.path.expanduser(root_dir)
        self.rgb_dir = os.path.join(self.root, 'rgb')
        self.depth_dir = os.path.join(self.root, 'depth')
        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)

        # Files
        self.rgb_txt = open(os.path.join(self.root, 'rgb.txt'), 'w')
        self.depth_txt = open(os.path.join(self.root, 'depth.txt'), 'w')
        self.assoc_txt = open(os.path.join(self.root, 'associations.txt'), 'w')
        self.imu_raw_txt = open(os.path.join(self.root, 'imu.txt'), 'w')
        self.imu_synced_txt = open(os.path.join(self.root, 'imu_synced.txt'), 'w')

        # Headers
        self.rgb_txt.write("# timestamp filename\n")
        self.depth_txt.write("# timestamp filename\n")
        self.assoc_txt.write("# timestamp_rgb rgb_file timestamp_depth depth_file\n")
        self.imu_raw_txt.write("# timestamp ax ay az gx gy gz\n")
        self.imu_synced_txt.write("# timestamp ax ay az gx gy gz (synced with RGB-D)\n")

        self.frame_id = 0
        self.imu_raw_count = 0
        self.imu_synced_count = 0
        self.get_logger().info(f"TUM RGBD+IMU logger root: {self.root}")

        # Subscribers
        self.rgb_sub = Subscriber(self, Image, rgb_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)
        self.imu_sub = Subscriber(self, Imu, imu_topic)

        # --- Sync 1: RGB + Depth ---
        self.sync_rgbd = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=50,
            slop=slop_rgbd
        )
        self.sync_rgbd.registerCallback(self.synced_rgbd_callback)

        # --- Sync 2: RGB + Depth + IMU ---
        self.sync_rgbd_imu = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.imu_sub],
            queue_size=100,
            slop=slop_rgbd_imu
        )
        self.sync_rgbd_imu.registerCallback(self.synced_rgbd_imu_callback)

        # --- Raw IMU subscriber (independent) ---
        self.imu_raw_sub = self.create_subscription(Imu, imu_topic,
                                                    self.imu_raw_callback, 500)

    def _ros_ts_to_float(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    # --- RGB-D only ---
    def synced_rgbd_callback(self, rgb_msg, depth_msg):
        ts = self._ros_ts_to_float(rgb_msg.header)

        # Convert
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

        rgb_filename = f"{self.frame_id:06d}.png"
        depth_filename = f"{self.frame_id:06d}.png"

        cv2.imwrite(os.path.join(self.rgb_dir, rgb_filename), rgb)
        cv2.imwrite(os.path.join(self.depth_dir, depth_filename), depth)

        # Write logs
        self.rgb_txt.write(f"{ts:.6f} rgb/{rgb_filename}\n")
        self.depth_txt.write(f"{ts:.6f} depth/{depth_filename}\n")
        self.assoc_txt.write(f"{ts:.6f} rgb/{rgb_filename} {ts:.6f} depth/{depth_filename}\n")

        if self.frame_id % 50 == 0 and self.frame_id > 0:
            self.rgb_txt.flush()
            self.depth_txt.flush()
            self.assoc_txt.flush()
            self.get_logger().info(f"✅ Logged {self.frame_id} RGB-D frames so far")

        self.frame_id += 1

    # --- RGB-D-IMU combined (synced IMU) ---
    def synced_rgbd_imu_callback(self, rgb_msg, depth_msg, imu_msg):
        ts = self._ros_ts_to_float(imu_msg.header)
        ax, ay, az = imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z
        gx, gy, gz = imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z
        self.imu_synced_txt.write(f"{ts:.6f} {ax:.9f} {ay:.9f} {az:.9f} {gx:.9f} {gy:.9f} {gz:.9f}\n")

        self.imu_synced_count += 1
        if self.imu_synced_count % 200 == 0:
            self.imu_synced_txt.flush()
            self.get_logger().info(f"📡 Logged {self.imu_synced_count} IMU samples synced with RGB-D")

    # --- Raw IMU (all samples at 84 Hz) ---
    def imu_raw_callback(self, imu_msg: Imu):
        ts = self._ros_ts_to_float(imu_msg.header)
        ax, ay, az = imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z
        gx, gy, gz = imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z
        self.imu_raw_txt.write(f"{ts:.6f} {ax:.9f} {ay:.9f} {az:.9f} {gx:.9f} {gy:.9f} {gz:.9f}\n")

        self.imu_raw_count += 1
        if self.imu_raw_count % 500 == 0:
            self.imu_raw_txt.flush()
            self.get_logger().info(f"🌀 Logged {self.imu_raw_count} raw IMU samples")

    def destroy_node(self):
        try:
            self.rgb_txt.close()
            self.depth_txt.close()
            self.assoc_txt.close()
            self.imu_raw_txt.close()
            self.imu_synced_txt.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TUMRGBDLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

