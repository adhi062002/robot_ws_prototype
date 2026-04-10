# #!/usr/bin/env python3
# """
# TUM-style RGB-D logger for ROS2.

# Generates (in dataset_root):
#   - rgb/000000.png, rgb/000001.png, ...
#   - depth/000000.png (16-bit PNG)
#   - rgb.txt          (timestamp rgb/xxxxx.png)
#   - depth.txt        (timestamp depth/xxxxx.png)
#   - associations.txt (timestamp rgb/xxx.png timestamp depth/xxx.png)

# Usage:
#   - Edit topics/defaults below or pass args if you adapt this into a package.
#   - Run while your camera publishes the RGB + depth topics.
# """

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from message_filters import Subscriber, ApproximateTimeSynchronizer
# import numpy as np
# import cv2
# import os
# import argparse
# import math
# import time

# class TUMRGBDLogger(Node):
#     def __init__(self, root_dir, rgb_topic, depth_topic, slop=0.05):
#         super().__init__('tum_rgbd_logger')
#         self.bridge = CvBridge()

#         # Paths
#         self.root = os.path.expanduser(root_dir)
#         self.rgb_dir = os.path.join(self.root, 'rgb')
#         self.depth_dir = os.path.join(self.root, 'depth')
#         os.makedirs(self.rgb_dir, exist_ok=True)
#         os.makedirs(self.depth_dir, exist_ok=True)

#         self.rgb_txt = open(os.path.join(self.root, 'rgb.txt'), 'w')
#         self.depth_txt = open(os.path.join(self.root, 'depth.txt'), 'w')
#         self.assoc_txt = open(os.path.join(self.root, 'associations.txt'), 'w')

#         # Write header lines (TUM-like)
#         self.rgb_txt.write("# timestamp filename\n")
#         self.depth_txt.write("# timestamp filename\n")
#         self.assoc_txt.write("# timestamp_rgb rgb_file timestamp_depth depth_file\n")
#         self.rgb_txt.flush(); self.depth_txt.flush(); self.assoc_txt.flush()

#         self.frame_id = 0
#         self.get_logger().info(f"TUM RGBD logger root: {self.root}")

#         # message_filters subscribers and approximate sync
#         self.rgb_sub = Subscriber(self, Image, rgb_topic)
#         self.depth_sub = Subscriber(self, Image, depth_topic)
#         self.ats = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub],
#                                                queue_size=50,
#                                                slop=slop)
#         self.ats.registerCallback(self.synced_callback)

#     def _ros_ts_to_float(self, header):
#         return header.stamp.sec + header.stamp.nanosec * 1e-9

#     def _safe_convert_depth_to_uint16(self, depth_cv):
#         """
#         Convert depth cv image to uint16 (millimeters) suitable for TUM-style depth png.
#         Heuristics:
#           - if float and max < 20 -> treat as meters, multiply by 1000
#           - if float and max > 100 -> assume values already in mm, just cast
#           - if uint16 -> pass through
#         """
#         if depth_cv is None:
#             return None

#         if depth_cv.dtype == np.uint16:
#             return depth_cv
#         if np.issubdtype(depth_cv.dtype, np.floating):
#             maxv = np.nanmax(depth_cv)
#             if math.isnan(maxv) or maxv <= 0:
#                 # fallback
#                 return np.nan_to_num(depth_cv).astype(np.uint16)
#             if maxv < 20.0:
#                 # meters -> convert to mm
#                 depth_mm = (depth_cv * 1000.0).astype(np.uint16)
#             else:
#                 # already in mm but float
#                 depth_mm = np.nan_to_num(depth_cv).astype(np.uint16)
#             return depth_mm
#         # other integer types
#         return depth_cv.astype(np.uint16)

#     def synced_callback(self, rgb_msg: Image, depth_msg: Image):
#         try:
#             rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"cv_bridge rgb exception: {e}")
#             return

#         try:
#             depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"cv_bridge depth exception: {e}")
#             return

#         # fix depth dtype -> uint16 (mm)
#         depth_mm = self._safe_convert_depth_to_uint16(depth)

#         # small cleanup optional (comment/uncomment as needed)
#         # depth_mm = cv2.medianBlur(depth_mm, 5)

#         # timestamps
#         ts_rgb = self._ros_ts_to_float(rgb_msg.header)
#         ts_depth = self._ros_ts_to_float(depth_msg.header)

#         # filename by frame index
#         fname = f"{self.frame_id:06d}.png"
#         rgb_path = os.path.join(self.rgb_dir, fname)
#         depth_path = os.path.join(self.depth_dir, fname)

#         # write files
#         success_rgb = cv2.imwrite(rgb_path, rgb)
#         if not success_rgb:
#             self.get_logger().error(f"Failed to write RGB {rgb_path}")
#             return

#         success_depth = cv2.imwrite(depth_path, depth_mm)
#         if not success_depth:
#             self.get_logger().error(f"Failed to write depth {depth_path}")
#             return

#         # write index lines
#         self.rgb_txt.write(f"{ts_rgb:.6f} rgb/{fname}\n")
#         self.depth_txt.write(f"{ts_depth:.6f} depth/{fname}\n")
#         self.assoc_txt.write(f"{ts_rgb:.6f} rgb/{fname} {ts_depth:.6f} depth/{fname}\n")

#         # flush occasionally
#         if self.frame_id % 10 == 0:
#             self.rgb_txt.flush()
#             self.depth_txt.flush()
#             self.assoc_txt.flush()

#         if self.frame_id % 50 == 0:
#             self.get_logger().info(f"Recorded frame {self.frame_id}")

#         self.frame_id += 1

#     def destroy_node(self):
#         # close files
#         try:
#             self.rgb_txt.flush(); self.rgb_txt.close()
#             self.depth_txt.flush(); self.depth_txt.close()
#             self.assoc_txt.flush(); self.assoc_txt.close()
#         except Exception:
#             pass
#         super().destroy_node()


# def main():
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--root', type=str, default='~/datasets/orb_hp60c_rgbd',
#                         help='dataset root folder')
#     parser.add_argument('--rgb_topic', type=str,
#                         default='/ascamera_hp60c/camera_publisher/rgb0/image',
#                         help='RGB topic')
#     parser.add_argument('--depth_topic', type=str,
#                         default='/ascamera_hp60c/camera_publisher/depth0/image_raw',
#                         help='Depth topic')
#     parser.add_argument('--slop', type=float, default=0.05,
#                         help='Approx sync slop (seconds)')
#     args = parser.parse_args()

#     rclpy.init()
#     node = TUMRGBDLogger(args.root, args.rgb_topic, args.depth_topic, slop=args.slop)
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3
# import os
# import cv2
# import rclpy
# from rclpy.node import Node
# from cv_bridge import CvBridge
# from message_filters import Subscriber, ApproximateTimeSynchronizer
# from sensor_msgs.msg import Image, Imu


# class TUMRGBDLogger(Node):
#     def __init__(self,
#                  root_dir="~/datasets/tum_dataset",
#                  rgb_topic="/ascamera_hp60c/camera_publisher/rgb0/image",
#                  depth_topic="/ascamera_hp60c/camera_publisher/depth0/image",
#                  imu_topic="/imu/data_raw",
#                  slop=0.05):

#         super().__init__('tum_rgbd_logger')
#         self.bridge = CvBridge()

#         # Directories
#         self.root = os.path.expanduser(root_dir)
#         self.rgb_dir = os.path.join(self.root, 'rgb')
#         self.depth_dir = os.path.join(self.root, 'depth')
#         os.makedirs(self.rgb_dir, exist_ok=True)
#         os.makedirs(self.depth_dir, exist_ok=True)

#         # Files
#         self.rgb_txt = open(os.path.join(self.root, 'rgb.txt'), 'w')
#         self.depth_txt = open(os.path.join(self.root, 'depth.txt'), 'w')
#         self.assoc_txt = open(os.path.join(self.root, 'associations.txt'), 'w')
#         self.imu_txt = open(os.path.join(self.root, 'imu.txt'), 'w')

#         # Headers
#         self.rgb_txt.write("# timestamp filename\n")
#         self.depth_txt.write("# timestamp filename\n")
#         self.assoc_txt.write("# timestamp_rgb rgb_file timestamp_depth depth_file\n")
#         self.imu_txt.write("# timestamp ax ay az gx gy gz\n")

#         self.frame_id = 0
#         self.get_logger().info(f"TUM RGBD+IMU logger root: {self.root}")

#         # Subscribers
#         self.rgb_sub = Subscriber(self, Image, rgb_topic)
#         self.depth_sub = Subscriber(self, Image, depth_topic)
#         self.ats = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub],
#                                                queue_size=50,
#                                                slop=slop)
#         self.ats.registerCallback(self.synced_callback)

#         # IMU
#         self.imu_sub = self.create_subscription(Imu, imu_topic,
#                                                 self.imu_callback, 200)

#     def _ros_ts_to_float(self, header):
#         return header.stamp.sec + header.stamp.nanosec * 1e-9

#     def synced_callback(self, rgb_msg, depth_msg):
#         ts = self._ros_ts_to_float(rgb_msg.header)

#         # Convert
#         rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
#         depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

#         rgb_filename = f"{self.frame_id:06d}.png"
#         depth_filename = f"{self.frame_id:06d}.png"

#         rgb_path = os.path.join(self.rgb_dir, rgb_filename)
#         depth_path = os.path.join(self.depth_dir, depth_filename)

#         cv2.imwrite(rgb_path, rgb)
#         cv2.imwrite(depth_path, depth)

#         # Write logs
#         self.rgb_txt.write(f"{ts:.6f} rgb/{rgb_filename}\n")
#         self.depth_txt.write(f"{ts:.6f} depth/{depth_filename}\n")
#         self.assoc_txt.write(f"{ts:.6f} rgb/{rgb_filename} {ts:.6f} depth/{depth_filename}\n")

#         if self.frame_id % 50 == 0:
#             self.rgb_txt.flush()
#             self.depth_txt.flush()
#             self.assoc_txt.flush()
#             self.get_logger().info(f"Logged {self.frame_id} frames...")

#         self.frame_id += 1

#     def imu_callback(self, imu_msg: Imu):
#         ts = self._ros_ts_to_float(imu_msg.header)
#         ax, ay, az = imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z
#         gx, gy, gz = imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z
#         self.imu_txt.write(f"{ts:.6f} {ax:.9f} {ay:.9f} {az:.9f} {gx:.9f} {gy:.9f} {gz:.9f}\n")

#         if self.frame_id % 200 == 0:
#             self.imu_txt.flush()

#     def destroy_node(self):
#         try:
#             self.rgb_txt.close()
#             self.depth_txt.close()
#             self.assoc_txt.close()
#             self.imu_txt.close()
#         except Exception:
#             pass
#         super().destroy_node()


# def main(args=None):
#     rclpy.init(args=args)
#     node = TUMRGBDLogger()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
import os
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image


class TUMRGBDLogger(Node):
    def __init__(self,
                 root_dir="~/data_3",
                 rgb_topic="/ascamera_hp60c/camera_publisher/rgb0/image",
                 depth_topic="/ascamera_hp60c/camera_publisher/depth0/image_raw",
                 slop=0.05):

        super().__init__('tum_rgbd_logger')
        self.bridge = CvBridge()

        # Dataset directories
        self.root = os.path.expanduser(root_dir)
        self.rgb_dir = os.path.join(self.root, 'rgb')
        self.depth_dir = os.path.join(self.root, 'depth')
        os.makedirs(self.rgb_dir, exist_ok=True)
        os.makedirs(self.depth_dir, exist_ok=True)

        # Metadata files
        self.rgb_txt = open(os.path.join(self.root, 'rgb.txt'), 'w')
        self.depth_txt = open(os.path.join(self.root, 'depth.txt'), 'w')
        self.assoc_txt = open(os.path.join(self.root, 'associations.txt'), 'w')

        # Headers
        self.rgb_txt.write("# timestamp filename\n")
        self.depth_txt.write("# timestamp filename\n")
        self.assoc_txt.write("# timestamp_rgb rgb_file timestamp_depth depth_file\n")

        self.frame_id = 0
        self.get_logger().info(f"TUM RGBD logger root: {self.root}")

        # Subscribers with sync
        self.rgb_sub = Subscriber(self, Image, rgb_topic, qos_profile=10)
        self.depth_sub = Subscriber(self, Image, depth_topic, qos_profile=10)
        self.ats = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub],
                                               queue_size=50,
                                               slop=slop)
        self.ats.registerCallback(self.synced_callback)

    def _ros_ts_to_float(self, header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def synced_callback(self, rgb_msg, depth_msg):
        ts = self._ros_ts_to_float(rgb_msg.header)

        # Convert ROS → OpenCV
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")

        rgb_filename = f"{self.frame_id:06d}.png"
        depth_filename = f"{self.frame_id:06d}.png"

        rgb_path = os.path.join(self.rgb_dir, rgb_filename)
        depth_path = os.path.join(self.depth_dir, depth_filename)

        cv2.imwrite(rgb_path, rgb)
        cv2.imwrite(depth_path, depth)

        # Write metadata
        self.rgb_txt.write(f"{ts:.6f} rgb/{rgb_filename}\n")
        self.depth_txt.write(f"{ts:.6f} depth/{depth_filename}\n")
        self.assoc_txt.write(f"{ts:.6f} rgb/{rgb_filename} {ts:.6f} depth/{depth_filename}\n")

        if self.frame_id % 50 == 0:
            self.rgb_txt.flush()
            self.depth_txt.flush()
            self.assoc_txt.flush()
            self.get_logger().info(f"Logged {self.frame_id} frames...")

        self.frame_id += 1

    def destroy_node(self):
        try:
            self.rgb_txt.close()
            self.depth_txt.close()
            self.assoc_txt.close()
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
