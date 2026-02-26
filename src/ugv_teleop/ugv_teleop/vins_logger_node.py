#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import message_filters
# from sensor_msgs.msg import Image
# from nav_msgs.msg import Odometry
# from cv_bridge import CvBridge
# import numpy as np
# import open3d as o3d
# import cv2
# import csv
# import os
# from datetime import datetime


# class TSDFFusionNode(Node):
#     def __init__(self):
#         super().__init__('tsdf_fusion_with_odometry')

#         # ---- Parameters ----
#         self.declare_parameter("rgb_topic", "/ascamera_hp60c/camera_publisher/rgb0/image")
#         self.declare_parameter("depth_topic", "/ascamera_hp60c/camera_publisher/depth0/image_raw")
#         self.declare_parameter("odom_topic", "/vio/odom")
#         self.declare_parameter("camera_frame", "cam0")

#         rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
#         depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
#         odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
#         self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

#         # ---- Static extrinsic: body(imu) → cam0 ----
#         self.T_body_cam = np.array([
#             [-0.02156132, -0.99483675, -0.09917134,  0.03432751],
#             [-0.25782609,  0.10137213, -0.96085868,  0.03072772],
#             [ 0.96595073,  0.00485158, -0.25868058, -0.07285357],
#             [ 0.,          0.,          0.,          1.        ]
#         ])

#         # ---- Intrinsics (replace with your calibrated values) ----
#         fx, fy, cx, cy = 525.0, 525.0, 319.5, 239.5
#         width, height = 640, 480
#         self.intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

#         # ---- TSDF volume ----
#         self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
#             voxel_length=0.02,
#             sdf_trunc=0.04,
#             color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
#         )

#         # ---- Subscribers ----
#         self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
#         self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
#         self.odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)

#         self.ts = message_filters.ApproximateTimeSynchronizer(
#             [self.rgb_sub, self.depth_sub, self.odom_sub],
#             queue_size=10, slop=0.03
#         )
#         self.ts.registerCallback(self.sync_callback)

#         # ---- Helpers ----
#         self.bridge = CvBridge()
#         self.frame_id = 0
#         self.pose_records = []
#         self.global_pcd = o3d.geometry.PointCloud()
#         self.depth_trunc = 2.0
#         self.min_valid_depth_pixels = 500  # skip frames with almost no depth

#         self.get_logger().info("TSDF fusion node initialized (RGBD + odometry_rect).")

#     def _depth_msg_to_numpy(self, depth_msg, depth_cv_raw):
#         """Convert depth cv2 image to numpy with correct dtype + scale."""
#         encoding = depth_msg.encoding.lower()
#         if encoding in ("16uc1", "mono16"):
#             depth_np = np.asarray(depth_cv_raw, dtype=np.uint16)
#             depth_scale = 1000.0
#         elif encoding in ("32fc1", "float32"):
#             depth_np = np.asarray(depth_cv_raw, dtype=np.float32)
#             depth_scale = 1.0
#         else:
#             # fallback: try uint16
#             depth_np = np.asarray(depth_cv_raw).astype(np.uint16)
#             depth_scale = 1000.0
#         return depth_np, depth_scale

#     def odom_to_matrix(self, odom: Odometry):
#         """Convert nav_msgs/Odometry pose to 4x4 transformation matrix."""
#         pose = odom.pose.pose
#         tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
#         qx, qy, qz, qw = (
#             pose.orientation.x,
#             pose.orientation.y,
#             pose.orientation.z,
#             pose.orientation.w,
#         )

#         norm = qw*qw + qx*qx + qy*qy + qz*qz
#         s = 1.0 / np.sqrt(norm)
#         qw, qx, qy, qz = qw*s, qx*s, qy*s, qz*s
#         R = np.array([
#             [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
#             [2*(qx*qy+qz*qw),     1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
#             [2*(qx*qz-qy*qw),     2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]
#         ], dtype=np.float64)
#         T = np.eye(4)
#         T[:3, :3] = R
#         T[:3, 3] = np.array([tx, ty, tz])
#         return T

#     def sync_callback(self, rgb_msg: Image, depth_msg: Image, odom_msg: Odometry):
#         # --- Convert RGB ---
#         try:
#             rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"RGB cv_bridge failed: {e}")
#             return

#         # --- Convert Depth (robust) ---
#         try:
#             depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Depth cv_bridge passthrough failed: {e}")
#             try:
#                 depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
#             except Exception as e2:
#                 self.get_logger().error(f"Depth cv_bridge fallback failed: {e2}")
#                 return

#         try:
#             depth_np, depth_scale = self._depth_msg_to_numpy(depth_msg, depth_cv_raw)
#         except Exception as e:
#             self.get_logger().error(f"Depth -> numpy sanitization failed: {e}")
#             return

#         valid = int(np.count_nonzero(depth_np))
#         if valid < self.min_valid_depth_pixels:
#             self.get_logger().warn(f"Skipping frame {self.frame_id}: too few valid depth pixels ({valid}).")
#             return
        
#         # resize RGB if mismatch
#         if rgb_cv.shape[0] != self.intrinsics.height or rgb_cv.shape[1] != self.intrinsics.width:
#             rgb_cv = cv2.resize(rgb_cv, (self.intrinsics.width, self.intrinsics.height))

#         # --- Build Open3D RGB + Depth ---
#         try:
#             color_o3d = o3d.geometry.Image(cv2.cvtColor(np.ascontiguousarray(rgb_cv), cv2.COLOR_BGR2RGB))
#             depth_o3d = o3d.geometry.Image(np.ascontiguousarray(depth_np))
#         except Exception as e:
#             self.get_logger().error(f"Open3D image creation failed: {e}")
#             return

#         try:
#             rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
#                 color_o3d, depth_o3d,
#                 depth_scale=depth_scale,
#                 depth_trunc=self.depth_trunc,
#                 convert_rgb_to_intensity=False
#             )
#         except Exception as e:
#             self.get_logger().error(f"RGBD creation failed: {e}")
#             return

#         # --- Get odometry pose ---
#         T_w_body = self.odom_to_matrix(odom_msg)
#         T_w_cam = T_w_body @ self.T_body_cam

#         # --- Fuse ---
#         try:
#             pcd_cam = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)
#             flip_rosopt_to_open3d = np.array([
#                 [1, 0, 0, 0],
#                 [0,-1, 0, 0],
#                 [0, 0,-1, 0],
#                 [0, 0, 0, 1]
#             ])
#             pcd_cam.transform(flip_rosopt_to_open3d)
#             pcd_cam.transform(T_w_cam)
#             self.global_pcd += pcd_cam
#         except Exception as e:
#             self.get_logger().warn(f"Open3D processing failed: {e}")

#         extrinsic = np.linalg.inv(T_w_cam)
#         self.volume.integrate(rgbd, self.intrinsics, extrinsic)

#         # --- Save pose record ---
#         row = [self.frame_id, f"{rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec:09d}"]
#         row.extend(T_w_cam.reshape(-1).tolist())
#         self.pose_records.append(row)
#         self.frame_id += 1

#     def save_results(self):
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         out_dir = f"tsdf_output_{timestamp}"
#         os.makedirs(out_dir, exist_ok=True)

#         mesh = self.volume.extract_triangle_mesh()
#         mesh.compute_vertex_normals()
#         o3d.io.write_triangle_mesh(os.path.join(out_dir, "mesh.ply"), mesh)
        
#         o3d.io.write_point_cloud(os.path.join(out_dir, "global_pcd.ply"), self.global_pcd)

#         with open(os.path.join(out_dir, "poses.csv"), "w") as f:
#             writer = csv.writer(f)
#             writer.writerow(["frame_id", "timestamp"] + [f"T{i}{j}" for i in range(4) for j in range(4)])
#             writer.writerows(self.pose_records)

#         self.get_logger().info(f"Saved results to {out_dir}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = TSDFFusionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.save_results()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
#!/usr/bin/env python3




# import rclpy
# from rclpy.node import Node
# import message_filters
# from sensor_msgs.msg import Image
# from nav_msgs.msg import Odometry
# from cv_bridge import CvBridge
# import numpy as np
# import open3d as o3d
# import cv2
# import csv
# import os
# from datetime import datetime

# # === NEW imports ===
# import threading
# import time
# from copy import deepcopy

# class TSDFFusionNode(Node):
#     def __init__(self):
#         super().__init__('tsdf_fusion_with_odometry')

#         # ---- Parameters ----
#         self.declare_parameter("rgb_topic", "/ascamera_hp60c/camera_publisher/rgb0/image")
#         self.declare_parameter("depth_topic", "/ascamera_hp60c/camera_publisher/depth0/image_raw")
#         self.declare_parameter("odom_topic", "/odometry_rect")
#         self.declare_parameter("camera_frame", "cam0")
#         # new: buffer size and icp settings
#         self.declare_parameter("pcd_buffer_size", 10)  # number of recent pcds to keep for ICP
#         self.declare_parameter("icp_voxel_size", 0.02)
#         self.declare_parameter("icp_max_correspondence_distance", 0.05)
#         self.declare_parameter("icp_downsample_voxel", 0.02)
#         self.declare_parameter("icp_run_interval", 1.0)  # seconds between ICP runs (if no new frames waits up to this)

#         rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
#         depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
#         odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
#         self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

#         # read icp params
#         self.pcd_buffer_size = self.get_parameter("pcd_buffer_size").get_parameter_value().integer_value
#         self.icp_voxel_size = self.get_parameter("icp_voxel_size").get_parameter_value().double_value
#         self.icp_max_corr = self.get_parameter("icp_max_correspondence_distance").get_parameter_value().double_value
#         self.icp_downsample_voxel = self.get_parameter("icp_downsample_voxel").get_parameter_value().double_value
#         self.icp_run_interval = self.get_parameter("icp_run_interval").get_parameter_value().double_value

#         # ---- Static extrinsic: body(imu) → cam0 ----
#         self.T_body_cam = np.array([
#             [-0.02156132, -0.99483675, -0.09917134,  0.03432751],
#             [-0.25782609,  0.10137213, -0.96085868,  0.03072772],
#             [ 0.96595073,  0.00485158, -0.25868058, -0.07285357],
#             [ 0.,          0.,          0.,          1.        ]
#         ])

#         # ---- Intrinsics (replace with your calibrated values) ----
#         fx, fy, cx, cy = 525.0, 525.0, 319.5, 239.5
#         width, height = 640, 480
#         self.intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

#         # ---- TSDF volume ----
#         # self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
#         #     voxel_length=0.02,
#         #     sdf_trunc=0.04,
#         #     color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
#         # )

#         # ---- Subscribers ----
#         self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
#         self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
#         self.odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)

#         self.ts = message_filters.ApproximateTimeSynchronizer(
#             [self.rgb_sub, self.depth_sub, self.odom_sub],
#             queue_size=10, slop=0.01
#         )
#         self.ts.registerCallback(self.sync_callback)

#         # ---- Helpers ----
#         self.bridge = CvBridge()
#         self.frame_id = 0
#         self.pose_records = []
#         self.global_pcd = o3d.geometry.PointCloud()
#         self.depth_trunc = 2.0
#         self.min_valid_depth_pixels = 500  # skip frames with almost no depth

#         # === ICP buffer & threading ===
#         self.pcd_buffer = []  # list of recent pointclouds (o3d.geometry.PointCloud)
#         self.buffer_lock = threading.Lock()
#         self.new_frame_event = threading.Event()
#         self.shutdown_event = threading.Event()
#         # start ICP thread
#         self.icp_thread = threading.Thread(target=self._icp_worker, daemon=True)
#         self.icp_thread.start()

#         self.get_logger().info("TSDF fusion node initialized (RGBD + odometry_rect). ICP worker started.")

#     def _depth_msg_to_numpy(self, depth_msg, depth_cv_raw):
#         """Convert depth cv2 image to numpy with correct dtype + scale."""
#         encoding = depth_msg.encoding.lower()
#         if encoding in ("16uc1", "mono16"):
#             depth_np = np.asarray(depth_cv_raw, dtype=np.uint16)
#             depth_scale = 1000.0
#         elif encoding in ("32fc1", "float32"):
#             depth_np = np.asarray(depth_cv_raw, dtype=np.float32)
#             depth_scale = 1.0
#         else:
#             # fallback: try uint16
#             depth_np = np.asarray(depth_cv_raw).astype(np.uint16)
#             depth_scale = 1000.0
#         return depth_np, depth_scale

#     def odom_to_matrix(self, odom: Odometry):
#         """Convert nav_msgs/Odometry pose to 4x4 transformation matrix."""
#         pose = odom.pose.pose
#         tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
#         qx, qy, qz, qw = (
#             pose.orientation.x,
#             pose.orientation.y,
#             pose.orientation.z,
#             pose.orientation.w,
#         )

#         norm = qw*qw + qx*qx + qy*qy + qz*qz
#         s = 1.0 / np.sqrt(norm)
#         qw, qx, qy, qz = qw*s, qx*s, qy*s, qz*s
#         R = np.array([
#             [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
#             [2*(qx*qy+qz*qw),     1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
#             [2*(qx*qz-qy*qw),     2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]
#         ], dtype=np.float64)
#         T = np.eye(4)
#         T[:3, :3] = R
#         T[:3, 3] = np.array([tx, ty, tz])
#         return T

#     def sync_callback(self, rgb_msg: Image, depth_msg: Image, odom_msg: Odometry):
#         # --- Convert RGB ---
#         try:
#             rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"RGB cv_bridge failed: {e}")
#             return

#         # --- Convert Depth (robust) ---
#         try:
#             depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Depth cv_bridge passthrough failed: {e}")
#             try:
#                 depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
#             except Exception as e2:
#                 self.get_logger().error(f"Depth cv_bridge fallback failed: {e2}")
#                 return

#         try:
#             depth_np, depth_scale = self._depth_msg_to_numpy(depth_msg, depth_cv_raw)
#         except Exception as e:
#             self.get_logger().error(f"Depth -> numpy sanitization failed: {e}")
#             return

#         valid = int(np.count_nonzero(depth_np))
#         if valid < self.min_valid_depth_pixels:
#             self.get_logger().warn(f"Skipping frame {self.frame_id}: too few valid depth pixels ({valid}).")
#             return
        
#         # resize RGB if mismatch
#         if rgb_cv.shape[0] != self.intrinsics.height or rgb_cv.shape[1] != self.intrinsics.width:
#             rgb_cv = cv2.resize(rgb_cv, (self.intrinsics.width, self.intrinsics.height))

#         # --- Build Open3D RGB + Depth ---
#         try:
#             color_o3d = o3d.geometry.Image(cv2.cvtColor(np.ascontiguousarray(rgb_cv), cv2.COLOR_BGR2RGB))
#             depth_o3d = o3d.geometry.Image(np.ascontiguousarray(depth_np))
#         except Exception as e:
#             self.get_logger().error(f"Open3D image creation failed: {e}")
#             return

#         try:
#             rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
#                 color_o3d, depth_o3d,
#                 depth_scale=depth_scale,
#                 depth_trunc=self.depth_trunc,
#                 convert_rgb_to_intensity=False
#             )
#         except Exception as e:
#             self.get_logger().error(f"RGBD creation failed: {e}")
#             return

#         # --- Get odometry pose ---
#         T_w_body = self.odom_to_matrix(odom_msg)
#         T_w_cam = T_w_body @ self.T_body_cam

#         # --- Fuse ---
#         try:
#             pcd_cam = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)
#             flip_rosopt_to_open3d = np.array([
#                 [1, 0, 0, 0],
#                 [0,-1, 0, 0],
#                 [0, 0,-1, 0],
#                 [0, 0, 0, 1]
#             ])
#             pcd_cam.transform(flip_rosopt_to_open3d)
#             pcd_cam.transform(T_w_cam)
#             #self.global_pcd += pcd_cam
#         except Exception as e:
#             self.get_logger().warn(f"Open3D processing failed: {e}")

#         # extrinsic = np.linalg.inv(T_w_cam)
#         # self.volume.integrate(rgbd, self.intrinsics, extrinsic)

#         # --- Save pose record ---
#         row = [self.frame_id, f"{rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec:09d}"]
#         row.extend(T_w_cam.reshape(-1).tolist())
#         self.pose_records.append(row)

#         # === NEW: add downsampled copy of pcd_cam to buffer for ICP ===
#         try:
#             # create a copy / downsample to keep memory bounded
#             tmp = pcd_cam.voxel_down_sample(voxel_size=self.icp_downsample_voxel)
#             # open3d pointcloud has clone in recent versions; fallback to deepcopy
#             try:
#                 pcd_copy = tmp.clone()
#             except Exception:
#                 pcd_copy = deepcopy(tmp)
#             with self.buffer_lock:
#                 self.pcd_buffer.append(pcd_copy)
#                 # keep buffer bounded: drop oldest if needed
#                 if len(self.pcd_buffer) > self.pcd_buffer_size:
#                     # drop oldest
#                     self.pcd_buffer.pop(0)
#             # signal ICP thread
#             self.new_frame_event.set()
#         except Exception as e:
#             self.get_logger().warn(f"Failed to push pcd to ICP buffer: {e}")

#         self.frame_id += 1


    # # === NEW: ICP worker that registers the buffer into a merged cloud, then aligns to global_pcd ===
    # def _icp_worker(self):
    #     """
    #     Background worker: waits for new frames, then performs ICP on a snapshot of the buffer,
    #     producing a merged point cloud. That result is then aligned to the global_pcd using
    #     multi-scale ICP with the last known transformation as initialization.
    #     """
    #     self.get_logger().info("ICP worker thread started.")
    #     last_global_transform = np.eye(4)

    #     # choose robust kernel type (default: Tukey)
    #     loss_type = getattr(self, "icp_loss_type", "huber").lower()
    #     def make_loss(scale):
    #         if loss_type == "tukey":
    #             return o3d.pipelines.registration.TukeyLoss(scale)
    #         elif loss_type == "huber":
    #             return o3d.pipelines.registration.HuberLoss(scale)
    #         else:  # fallback L2
    #             return None

    #     while not self.shutdown_event.is_set():
    #         # Wait until a new frame arrives or timeout
    #         self.new_frame_event.wait(self.icp_run_interval)
    #         self.new_frame_event.clear()
    #         if self.shutdown_event.is_set():
    #             break

    #         # copy buffer snapshot
    #         with self.buffer_lock:
    #             buffer_snapshot = [pcd.clone() if hasattr(pcd, "clone") else deepcopy(pcd) for pcd in self.pcd_buffer]

    #         if len(buffer_snapshot) == 0:
    #             continue

    #         try:
    #             # --- Step 1: merge buffer clouds into a local "merged" ---
    #             if len(buffer_snapshot) == 1:
    #                 merged = buffer_snapshot[0]
    #             else:
    #                 target = buffer_snapshot[0]
    #                 target_down = target.voxel_down_sample(self.icp_voxel_size)
    #                 target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30))

    #                 merged = target_down
    #                 for i in range(1, len(buffer_snapshot)):
    #                     source = buffer_snapshot[i]
    #                     source_down = source.voxel_down_sample(self.icp_voxel_size)
    #                     source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30))

    #                     reg = o3d.pipelines.registration.registration_icp(
    #                         source_down, merged,
    #                         self.icp_max_corr,
    #                         np.eye(4),
    #                         o3d.pipelines.registration.TransformationEstimationPointToPlane(make_loss(self.icp_voxel_size))
    #                     )
    #                     T = reg.transformation
    #                     source_transformed = deepcopy(source).transform(T)
    #                     merged += source_transformed
    #                     merged = merged.voxel_down_sample(self.icp_downsample_voxel)
    #                     merged.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_downsample_voxel*2, max_nn=30))

    #             # --- Step 2: align merged to global_pcd ---
    #             with self.buffer_lock:
    #                 global_copy = None
    #                 if len(self.global_pcd.points) > 0:
    #                     global_copy = self.global_pcd.voxel_down_sample(self.icp_voxel_size)
    #                     global_copy.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30))

    #             if global_copy is not None:
    #                 # multi-scale ICP (coarse → fine)
    #                 voxel_radii = [self.icp_voxel_size*4, self.icp_voxel_size*2, self.icp_voxel_size]
    #                 max_iters = [50, 30, 14]
    #                 current_trans = last_global_transform
    #                 for scale, max_iter in zip(voxel_radii, max_iters):
    #                     src_down = merged.voxel_down_sample(scale)
    #                     tgt_down = global_copy.voxel_down_sample(scale)
    #                     src_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=scale*2, max_nn=30))
    #                     tgt_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=scale*2, max_nn=30))

    #                     reg = o3d.pipelines.registration.registration_icp(
    #                         src_down, tgt_down,
    #                         self.icp_max_corr*8,
    #                         current_trans,
    #                         o3d.pipelines.registration.TransformationEstimationPointToPlane(make_loss(scale)),
    #                         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
    #                     )
    #                     current_trans = reg.transformation

    #                 last_global_transform = current_trans
    #                 merged.transform(current_trans)

    #             # --- Step 3: update global_pcd ---
    #             with self.buffer_lock:
    #                 if len(self.global_pcd.points) == 0:
    #                     self.global_pcd = merged
    #                 else:
    #                     self.global_pcd += merged
    #                     try:
    #                         self.global_pcd = self.global_pcd.voxel_down_sample(self.icp_downsample_voxel)
    #                     except Exception:
    #                         pass

    #             self.get_logger().info(f"ICP worker: merged {len(buffer_snapshot)} clouds -> aligned to global (total points {len(self.global_pcd.points)}).")
    #         except Exception as e:
    #             self.get_logger().warn(f"ICP worker failed: {e}")
    #             time.sleep(0.1)

    #     self.get_logger().info("ICP worker thread exiting.")
    
    # === NEW: ICP worker that registers the buffer into a merged cloud, then aligns to global_pcd ===
#     def _icp_worker(self):
#         """
#         Background worker: waits for new frames, then performs ICP on a snapshot of the buffer,
#         producing a merged point cloud. That result is then aligned to the global_pcd using
#         multi-scale ICP with the last known transformation as initialization.
#         """
#         self.get_logger().info("ICP worker thread started.")
#         last_global_transform = np.eye(4)

#         # --- NEW: explicitly define separate loss makers ---
#         def make_local_loss(scale):
#             # Tukey for local merges (strict, crisp)
#             return o3d.pipelines.registration.TukeyLoss(scale)

#         def make_global_loss(scale):
#             # Huber for global alignment (softer, stable)
#             return o3d.pipelines.registration.HuberLoss(scale)

#         while not self.shutdown_event.is_set():
#             # Wait until a new frame arrives or timeout
#             self.new_frame_event.wait(self.icp_run_interval)
#             self.new_frame_event.clear()
#             if self.shutdown_event.is_set():
#                 break

#             # copy buffer snapshot
#             with self.buffer_lock:
#                 buffer_snapshot = [pcd.clone() if hasattr(pcd, "clone") else deepcopy(pcd) 
#                                    for pcd in self.pcd_buffer]

#             if len(buffer_snapshot) == 0:
#                 continue

#             try:
#                 # --- Step 1: merge buffer clouds into a local "merged" ---
#                 if len(buffer_snapshot) == 1:
#                     merged = buffer_snapshot[0]
#                 else:
#                     target = buffer_snapshot[0]
#                     target_down = target.voxel_down_sample(self.icp_voxel_size)
#                     target_down.estimate_normals(
#                         o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30))

#                     merged = target_down
#                     for i in range(1, len(buffer_snapshot)):
#                         source = buffer_snapshot[i]
#                         source_down = source.voxel_down_sample(self.icp_voxel_size)
#                         source_down.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30))

#                         reg = o3d.pipelines.registration.registration_icp(
#                             source_down, merged,
#                             self.icp_max_corr,
#                             np.eye(4),
#                             o3d.pipelines.registration.TransformationEstimationPointToPlane(
#                                 make_local_loss(self.icp_voxel_size))
#                         )
#                         T = reg.transformation
#                         source_transformed = deepcopy(source).transform(T)
#                         merged += source_transformed
#                         merged = merged.voxel_down_sample(self.icp_downsample_voxel)
#                         merged.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_downsample_voxel*2, max_nn=30))

#                 # --- Step 2: align merged to global_pcd ---
#                 with self.buffer_lock:
#                     global_copy = None
#                     if len(self.global_pcd.points) > 0:
#                         global_copy = self.global_pcd.voxel_down_sample(self.icp_voxel_size)
#                         global_copy.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30))

#                 if global_copy is not None:
#                     # multi-scale ICP (coarse → fine)
#                     voxel_radii = [self.icp_voxel_size*4, self.icp_voxel_size*2, self.icp_voxel_size]
#                     max_iters = [50, 30, 14]
#                     current_trans = last_global_transform
#                     for scale, max_iter in zip(voxel_radii, max_iters):
#                         src_down = merged.voxel_down_sample(scale)
#                         tgt_down = global_copy.voxel_down_sample(scale)
#                         src_down.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=scale*2, max_nn=30))
#                         tgt_down.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=scale*2, max_nn=30))

#                         reg = o3d.pipelines.registration.registration_icp(
#                             src_down, tgt_down,
#                             self.icp_max_corr*8,
#                             current_trans,
#                             o3d.pipelines.registration.TransformationEstimationPointToPlane(
#                                 make_global_loss(scale)),
#                             o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
#                         )
#                         current_trans = reg.transformation

#                     last_global_transform = current_trans
#                     merged.transform(current_trans)

#                 # --- Step 3: update global_pcd ---
#                 with self.buffer_lock:
#                     if len(self.global_pcd.points) == 0:
#                         self.global_pcd = merged
#                     else:
#                         self.global_pcd += merged
#                         try:
#                             self.global_pcd = self.global_pcd.voxel_down_sample(self.icp_downsample_voxel)
#                         except Exception:
#                             pass

#                 self.get_logger().info(
#                     f"ICP worker: merged {len(buffer_snapshot)} clouds -> aligned to global "
#                     f"(total points {len(self.global_pcd.points)}).")
#             except Exception as e:
#                 self.get_logger().warn(f"ICP worker failed: {e}")
#                 time.sleep(0.1)

#         self.get_logger().info("ICP worker thread exiting.")

#     def save_results(self):
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         out_dir = f"tsdf_output_{timestamp}"
#         os.makedirs(out_dir, exist_ok=True)

#         # mesh = self.volume.extract_triangle_mesh()
#         # mesh.compute_vertex_normals()
#         # o3d.io.write_triangle_mesh(os.path.join(out_dir, "mesh.ply"), mesh)
        
#         o3d.io.write_point_cloud(os.path.join(out_dir, "global_pcd.ply"), self.global_pcd)

#         with open(os.path.join(out_dir, "poses.csv"), "w") as f:
#             writer = csv.writer(f)
#             writer.writerow(["frame_id", "timestamp"] + [f"T{i}{j}" for i in range(4) for j in range(4)])
#             writer.writerows(self.pose_records)

#         self.get_logger().info(f"Saved results to {out_dir}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = TSDFFusionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # signal ICP worker to exit and wait for it
#         try:
#             node.shutdown_event.set()
#             node.new_frame_event.set()  # wake the thread if it's waiting
#             if node.icp_thread.is_alive():
#                 node.get_logger().info("Waiting for ICP thread to finish...")
#                 node.icp_thread.join(timeout=5.0)
#         except Exception:
#             pass

#         node.save_results()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()


# import rclpy
# from rclpy.node import Node
# import message_filters
# from sensor_msgs.msg import Image
# from nav_msgs.msg import Odometry
# from cv_bridge import CvBridge
# import numpy as np
# import open3d as o3d
# import cv2
# import csv
# import os
# from datetime import datetime

# # === NEW imports ===
# import threading
# import time
# from copy import deepcopy

# # optional kd-tree insertion
# try:
#     from scipy.spatial import cKDTree
#     _HAS_SCIPY = True
# except Exception:
#     _HAS_SCIPY = False

# class TSDFFusionNode(Node):
#     def __init__(self):
#         super().__init__('tsdf_fusion_with_odometry')

#         # ---- Parameters ----
#         self.declare_parameter("rgb_topic", "/ascamera_hp60c/camera_publisher/rgb0/image")
#         self.declare_parameter("depth_topic", "/ascamera_hp60c/camera_publisher/depth0/image_raw")
#         self.declare_parameter("odom_topic", "/odometry_rect")
#         self.declare_parameter("camera_frame", "cam0")
#         # new: buffer size and icp settings
#         self.declare_parameter("pcd_buffer_size", 12)  # number of recent pcds to keep for ICP
#         self.declare_parameter("icp_voxel_size", 0.02)
#         self.declare_parameter("icp_max_correspondence_distance", 0.05)
#         self.declare_parameter("icp_downsample_voxel", 0.02)
#         self.declare_parameter("icp_run_interval", 1.0)  # seconds between ICP runs (if no new frames waits up to this)

#         rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
#         depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
#         odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
#         self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

#         # read icp params
#         self.pcd_buffer_size = self.get_parameter("pcd_buffer_size").get_parameter_value().integer_value
#         self.icp_voxel_size = self.get_parameter("icp_voxel_size").get_parameter_value().double_value
#         self.icp_max_corr = self.get_parameter("icp_max_correspondence_distance").get_parameter_value().double_value
#         self.icp_downsample_voxel = self.get_parameter("icp_downsample_voxel").get_parameter_value().double_value
#         self.icp_run_interval = self.get_parameter("icp_run_interval").get_parameter_value().double_value

#         # ---- Static extrinsic: body(imu) → cam0 ----
#         self.T_body_cam = np.array([
#             [-0.02156132, -0.99483675, -0.09917134,  0.03432751],
#             [-0.25782609,  0.10137213, -0.96085868,  0.03072772],
#             [ 0.96595073,  0.00485158, -0.25868058, -0.07285357],
#             [ 0.,          0.,          0.,          1.        ]
#         ])

#         # ---- Intrinsics (replace with your calibrated values) ----
#         fx, fy, cx, cy = 525.0, 525.0, 319.5, 239.5
#         width, height = 640, 480
#         self.intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

#         # ---- TSDF volume ----
#         # self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
#         #     voxel_length=0.02,
#         #     sdf_trunc=0.04,
#         #     color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
#         # )

#         # ---- Subscribers ----
#         self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
#         self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
#         self.odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)

#         self.ts = message_filters.ApproximateTimeSynchronizer(
#             [self.rgb_sub, self.depth_sub, self.odom_sub],
#             queue_size=10, slop=0.01
#         )
#         self.ts.registerCallback(self.sync_callback)

#         # ---- Helpers ----
#         self.bridge = CvBridge()
#         self.frame_id = 0
#         self.pose_records = []
#         self.global_pcd = o3d.geometry.PointCloud()
#         self.depth_trunc = 2.0
#         self.min_valid_depth_pixels = 500  # skip frames with almost no depth

#         # === ICP buffer & threading ===
#         # buffer entries are dicts: {"pcd": o3d.geometry.PointCloud (camera frame), "pose": T_w_cam (world), "frame": id, "stamp": str}
#         self.pcd_buffer = []
#         self.buffer_lock = threading.Lock()
#         self.new_frame_event = threading.Event()
#         self.shutdown_event = threading.Event()
#         # start ICP thread
#         self.icp_thread = threading.Thread(target=self._icp_worker, daemon=True)
#         self.icp_thread.start()

#         self.get_logger().info("TSDF fusion node initialized (RGBD + odometry_rect). ICP worker started.")

#     def _depth_msg_to_numpy(self, depth_msg, depth_cv_raw):
#         """Convert depth cv2 image to numpy with correct dtype + scale."""
#         encoding = depth_msg.encoding.lower()
#         if encoding in ("16uc1", "mono16"):
#             depth_np = np.asarray(depth_cv_raw, dtype=np.uint16)
#             depth_scale = 1000.0
#         elif encoding in ("32fc1", "float32"):
#             depth_np = np.asarray(depth_cv_raw, dtype=np.float32)
#             depth_scale = 1.0
#         else:
#             # fallback: try uint16
#             depth_np = np.asarray(depth_cv_raw).astype(np.uint16)
#             depth_scale = 1000.0
#         return depth_np, depth_scale

#     def odom_to_matrix(self, odom: Odometry):
#         """Convert nav_msgs/Odometry pose to 4x4 transformation matrix."""
#         pose = odom.pose.pose
#         tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
#         qx, qy, qz, qw = (
#             pose.orientation.x,
#             pose.orientation.y,
#             pose.orientation.z,
#             pose.orientation.w,
#         )

#         norm = qw*qw + qx*qx + qy*qy + qz*qz
#         s = 1.0 / np.sqrt(norm)
#         qw, qx, qy, qz = qw*s, qx*s, qy*s, qz*s
#         R = np.array([
#             [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
#             [2*(qx*qy+qz*qw),     1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
#             [2*(qx*qz-qy*qw),     2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]
#         ], dtype=np.float64)
#         T = np.eye(4)
#         T[:3, :3] = R
#         T[:3, 3] = np.array([tx, ty, tz])
#         return T

#     def sync_callback(self, rgb_msg: Image, depth_msg: Image, odom_msg: Odometry):
#         # --- Convert RGB ---
#         try:
#             rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"RGB cv_bridge failed: {e}")
#             return

#         # --- Convert Depth (robust) ---
#         try:
#             depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Depth cv_bridge passthrough failed: {e}")
#             try:
#                 depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
#             except Exception as e2:
#                 self.get_logger().error(f"Depth cv_bridge fallback failed: {e2}")
#                 return

#         try:
#             depth_np, depth_scale = self._depth_msg_to_numpy(depth_msg, depth_cv_raw)
#         except Exception as e:
#             self.get_logger().error(f"Depth -> numpy sanitization failed: {e}")
#             return

#         valid = int(np.count_nonzero(depth_np))
#         if valid < self.min_valid_depth_pixels:
#             self.get_logger().warn(f"Skipping frame {self.frame_id}: too few valid depth pixels ({valid}).")
#             return
        
#         # resize RGB if mismatch
#         if rgb_cv.shape[0] != self.intrinsics.height or rgb_cv.shape[1] != self.intrinsics.width:
#             rgb_cv = cv2.resize(rgb_cv, (self.intrinsics.width, self.intrinsics.height))

#         # --- Build Open3D RGB + Depth ---
#         try:
#             color_o3d = o3d.geometry.Image(cv2.cvtColor(np.ascontiguousarray(rgb_cv), cv2.COLOR_BGR2RGB))
#             depth_o3d = o3d.geometry.Image(np.ascontiguousarray(depth_np))
#         except Exception as e:
#             self.get_logger().error(f"Open3D image creation failed: {e}")
#             return

#         try:
#             rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
#                 color_o3d, depth_o3d,
#                 depth_scale=depth_scale,
#                 depth_trunc=self.depth_trunc,
#                 convert_rgb_to_intensity=False
#             )
#         except Exception as e:
#             self.get_logger().error(f"RGBD creation failed: {e}")
#             return

#         # --- Get odometry pose ---
#         T_w_body = self.odom_to_matrix(odom_msg)
#         T_w_cam = T_w_body @ self.T_body_cam

#         # --- Fuse (note: keep pcd in camera frame, DO NOT apply T_w_cam here) ---
#         try:
#             pcd_cam = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)
#             flip_rosopt_to_open3d = np.array([
#                 [1, 0, 0, 0],
#                 [0,-1, 0, 0],
#                 [0, 0,-1, 0],
#                 [0, 0, 0, 1]
#             ])
#             # convert orientation convention but keep cloud in camera frame
#             pcd_cam.transform(flip_rosopt_to_open3d)
#             # convert the camera optical convention to Open3D camera frame and keep cloud in that frame
#             # compute corresponding world pose for this Open3D camera frame (so poses and clouds share convention)
#             # inv_flip = np.linalg.inv(flip_rosopt_to_open3d)
#             pose_open3d = T_w_cam @ flip_rosopt_to_open3d
#             # DO NOT immediately transform by T_w_cam here; store camera-frame cloud + world-pose in buffer
#             #self.global_pcd += pcd_cam
#         except Exception as e:
#             self.get_logger().warn(f"Open3D processing failed: {e}")

#         # extrinsic = np.linalg.inv(T_w_cam)
#         # self.volume.integrate(rgbd, self.intrinsics, extrinsic)

#         # --- Save pose record ---
#         row = [self.frame_id, f"{rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec:09d}"]
#         row.extend(T_w_cam.reshape(-1).tolist())
#         self.pose_records.append(row)

#         # === NEW: add downsampled copy of pcd_cam to buffer for ICP (store pcd in camera frame + pose) ===
#         try:
#             # create a copy / downsample to keep memory bounded
#             tmp = pcd_cam.voxel_down_sample(voxel_size=self.icp_downsample_voxel)
#             # open3d pointcloud has clone in recent versions; fallback to deepcopy
#             try:
#                 pcd_copy = tmp.clone()
#             except Exception:
#                 pcd_copy = deepcopy(tmp)

#             entry = {
#                 "pcd": pcd_copy,
#                 "pose": pose_open3d.copy(),
#                 "frame": self.frame_id,
#                 "stamp": f"{rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec:09d}"
#             }

#             with self.buffer_lock:
#                 self.pcd_buffer.append(entry)
#                 # keep buffer bounded: drop oldest if needed
#                 if len(self.pcd_buffer) > self.pcd_buffer_size:
#                     # drop oldest
#                     self.pcd_buffer.pop(0)
#             # signal ICP thread
#             self.new_frame_event.set()
#         except Exception as e:
#             self.get_logger().warn(f"Failed to push pcd to ICP buffer: {e}")

#         self.frame_id += 1


#     # === NEW: ICP worker that registers the buffer into a merged cloud, then aligns to global_pcd ===
#     def _icp_worker(self):
#         """
#         Background worker: waits for new frames, then performs ICP on a snapshot of the buffer,
#         producing a merged point cloud (all in world/Open3D-camera coords). That merged chunk is
#         then aligned to the global_pcd using multi-scale ICP and inserted into global_pcd.
#         """
#         self.get_logger().info("ICP worker thread started.")
#         last_global_transform = np.eye(4)

#         loss_type = getattr(self, "icp_loss_type", "tukey").lower()
#         def make_loss(scale):
#             if loss_type == "tukey":
#                 return o3d.pipelines.registration.TukeyLoss(scale)
#             elif loss_type == "huber":
#                 return o3d.pipelines.registration.HuberLoss(scale)
#             else:
#                 return None

#         while not self.shutdown_event.is_set():
#             self.new_frame_event.wait(self.icp_run_interval)
#             self.new_frame_event.clear()
#             if self.shutdown_event.is_set():
#                 break

#             with self.buffer_lock:
#                 buffer_snapshot = [
#                     {
#                         "pcd": (e["pcd"].clone() if hasattr(e["pcd"], "clone") else deepcopy(e["pcd"])),
#                         "pose": e["pose"].copy(),
#                         "frame": e["frame"],
#                         "stamp": e["stamp"]
#                     }
#                     for e in self.pcd_buffer
#                 ]
#                 buffer_snapshot.sort(key=lambda x: x['frame'])

#             if len(buffer_snapshot) == 0:
#                 continue

#             try:
#                 # Reference (target) is the first frame in buffer_snapshot
#                 target_item = buffer_snapshot[0]
#                 target_pcd = target_item["pcd"].voxel_down_sample(self.icp_voxel_size)
#                 target_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30))
#                 target_pose = target_item["pose"].copy()   # world -> Open3D camera (fixed)
#                 target_frame = target_item.get("frame", -1)

#                 # Create merged_world seeded by transforming target into world
#                 merged_world = deepcopy(target_pcd)
#                 merged_world.transform(target_pose)   # now merged_world is in world coords

#                 # For each remaining source, register it to the TARGET (not to a drifting merged frame)
#                 for i in range(1, len(buffer_snapshot)):
#                     src_item = buffer_snapshot[i]
#                     source = src_item["pcd"]
#                     source_pose = src_item["pose"].copy()  # world -> Open3D camera

#                     # downsample and normals for registration in camera frames
#                     src_down = source.voxel_down_sample(self.icp_voxel_size)
#                     src_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30))

#                     # initial guess: map source_cam -> target_cam
#                     T_init = np.linalg.inv(target_pose) @ source_pose

#                     # debug: magnitude of initial transform
#                     try:
#                         t_mag = np.linalg.norm(T_init[:3,3])
#                         rmat = T_init[:3,:3]
#                         angle = np.arccos(np.clip((np.trace(rmat)-1)/2, -1.0, 1.0))
#                         self.get_logger().debug(f"T_init {src_item.get('frame')} -> {target_frame}: trans={t_mag:.4f} rot_deg={np.degrees(angle):.2f}")
#                     except Exception:
#                         pass

#                     # coarse: point-to-point in target frame
#                     reg_p2p = o3d.pipelines.registration.registration_icp(
#                         src_down, target_pcd,
#                         max_correspondence_distance=self.icp_max_corr * 4.0,
#                         init=T_init,
#                         estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
#                     )

#                     # refine: point-to-plane in target frame
#                     reg = o3d.pipelines.registration.registration_icp(
#                         src_down, target_pcd,
#                         self.icp_max_corr,
#                         reg_p2p.transformation,
#                         o3d.pipelines.registration.TransformationEstimationPointToPlane(make_loss(self.icp_voxel_size)),
#                         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
#                     )

#                     self.get_logger().debug(f"pairwise reg frame {src_item['frame']} -> target {target_frame}: fitness={reg.fitness:.4f}, rmse={reg.inlier_rmse:.4f}")

#                     # transform source into target frame (reg.transformation maps source_cam -> target_cam)
#                     source_in_target = deepcopy(source)
#                     source_in_target.transform(reg.transformation)   # now in target_cam frame

#                     # then transform into world using target_pose (target_cam -> world)
#                     source_in_world = deepcopy(source_in_target)
#                     source_in_world.transform(target_pose)           # now in world coords

#                     # add to merged_world (world coords)
#                     merged_world += source_in_world

#                 # after all sources added, clean merged_world (voxel + normals)
#                 try:
#                     merged_world = merged_world.voxel_down_sample(self.icp_downsample_voxel)
#                     merged_world.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_downsample_voxel*2, max_nn=30))
#                 except Exception:
#                     pass

#                 # --- Step 2: align merged_world to global_pcd (if exists) using multi-scale ICP ---
#                 with self.buffer_lock:
#                     global_copy = None
#                     if len(self.global_pcd.points) > 0:
#                         global_copy = self.global_pcd.voxel_down_sample(self.icp_voxel_size)
#                         global_copy.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30))

#                 if global_copy is not None:
#                     voxel_radii = [self.icp_voxel_size*4, self.icp_voxel_size*2, self.icp_voxel_size]
#                     max_corrs = [self.icp_max_corr*4.0, self.icp_max_corr*2.0, self.icp_max_corr]
#                     current_trans = np.eye(4)   # merged_world already in world coords

#                     for idx, (scale, max_corr) in enumerate(zip(voxel_radii, max_corrs)):
#                         src_down = merged_world.voxel_down_sample(scale)
#                         tgt_down = global_copy.voxel_down_sample(scale)
#                         src_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=scale*2, max_nn=30))
#                         tgt_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=scale*2, max_nn=30))

#                         if idx == 0:
#                             reg = o3d.pipelines.registration.registration_icp(
#                                 src_down, tgt_down, max_corr, current_trans,
#                                 o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#                                 o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
#                             )
#                         else:
#                             reg = o3d.pipelines.registration.registration_icp(
#                                 src_down, tgt_down, max_corr, current_trans,
#                                 o3d.pipelines.registration.TransformationEstimationPointToPlane(make_loss(scale)),
#                                 o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
#                             )

#                         current_trans = reg.transformation
#                         self.get_logger().debug(f"global-scale {scale:.3f} fitness={reg.fitness:.4f} rmse={reg.inlier_rmse:.4f}")

#                     merged_world.transform(current_trans)

#                 # --- Step 3: merge into global map and clean duplicates/outliers ---
#                 with self.buffer_lock:
#                     if len(self.global_pcd.points) == 0:
#                         self.global_pcd = merged_world
#                     else:
#                         try:
#                             if _HAS_SCIPY:
#                                 glob_np = np.asarray(self.global_pcd.points)
#                                 merged_np = np.asarray(merged_world.points)
#                                 if glob_np.size == 0:
#                                     keep_idx = np.arange(len(merged_np))
#                                 else:
#                                     tree = cKDTree(glob_np)
#                                     dists, _ = tree.query(merged_np, k=1, n_jobs=-1)
#                                     thresh = max(self.icp_downsample_voxel * 0.5, 1e-3)
#                                     keep_idx = np.where(dists > thresh)[0]

#                                 if len(keep_idx) > 0:
#                                     new_pc = o3d.geometry.PointCloud()
#                                     new_pc.points = o3d.utility.Vector3dVector(merged_np[keep_idx])
#                                     try:
#                                         merged_colors = np.asarray(merged_world.colors)
#                                         new_pc.colors = o3d.utility.Vector3dVector(merged_colors[keep_idx])
#                                     except Exception:
#                                         pass
#                                     self.global_pcd += new_pc
#                             else:
#                                 self.global_pcd += merged_world

#                             # coarse voxel downsample then statistical outlier removal
#                             self.global_pcd = self.global_pcd.voxel_down_sample(self.icp_downsample_voxel)
#                             try:
#                                 filtered, ind = self.global_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
#                                 self.global_pcd = filtered
#                             except Exception:
#                                 pass

#                             try:
#                                 self.global_pcd.remove_duplicated_points()
#                             except Exception:
#                                 pass

#                         except Exception as e:
#                             try:
#                                 self.global_pcd += merged_world
#                                 self.global_pcd = self.global_pcd.voxel_down_sample(self.icp_downsample_voxel)
#                             except Exception:
#                                 pass

#                 self.get_logger().info(f"ICP worker: merged {len(buffer_snapshot)} -> global total {len(self.global_pcd.points)}")
#             except Exception as e:
#                 self.get_logger().warn(f"ICP worker failed: {e}")
#                 time.sleep(0.1)

#         self.get_logger().info("ICP worker thread exiting.")


#     def save_results(self):
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         out_dir = f"tsdf_output_{timestamp}"
#         os.makedirs(out_dir, exist_ok=True)

#         # mesh = self.volume.extract_triangle_mesh()
#         # mesh.compute_vertex_normals()
#         # o3d.io.write_triangle_mesh(os.path.join(out_dir, "mesh.ply"), mesh)
        
#         o3d.io.write_point_cloud(os.path.join(out_dir, "global_pcd.ply"), self.global_pcd)

#         with open(os.path.join(out_dir, "poses.csv"), "w") as f:
#             writer = csv.writer(f)
#             writer.writerow(["frame_id", "timestamp"] + [f"T{i}{j}" for i in range(4) for j in range(4)])
#             writer.writerows(self.pose_records)

#         self.get_logger().info(f"Saved results to {out_dir}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = TSDFFusionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # signal ICP worker to exit and wait for it
#         try:
#             node.shutdown_event.set()
#             node.new_frame_event.set()  # wake the thread if it's waiting
#             if node.icp_thread.is_alive():
#                 node.get_logger().info("Waiting for ICP thread to finish...")
#                 node.icp_thread.join(timeout=5.0)
#         except Exception:
#             pass

#         node.save_results()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()


# import rclpy
# from rclpy.node import Node
# import message_filters
# from sensor_msgs.msg import Image
# from nav_msgs.msg import Odometry
# from cv_bridge import CvBridge
# import numpy as np
# import open3d as o3d
# import cv2
# import csv
# import os
# from datetime import datetime

# # === NEW imports ===
# import threading
# import time
# from copy import deepcopy
# import math

# # optional kd-tree insertion
# try:
#     from scipy.spatial import cKDTree
#     _HAS_SCIPY = True
# except Exception:
#     _HAS_SCIPY = False

# # try import gtsam (optional)
# _HAS_GTSAM = False
# try:
#     import gtsam
#     _HAS_GTSAM = True
# except Exception:
#     # gtsam not available — we'll fallback to identity optimizer (no-op)
#     _HAS_GTSAM = False


# class TSDFFusionNode(Node):
#     def __init__(self):
#         super().__init__('tsdf_fusion_with_odometry')

#         # ---- Parameters ----
#         self.declare_parameter("rgb_topic", "/ascamera_hp60c/camera_publisher/rgb0/image")
#         self.declare_parameter("depth_topic", "/ascamera_hp60c/camera_publisher/depth0/image_raw")
#         self.declare_parameter("odom_topic", "/odometry_rect")
#         self.declare_parameter("camera_frame", "cam0")
#         # new: buffer size and icp settings
#         self.declare_parameter("pcd_buffer_size", 12)  # number of recent pcds to keep for ICP
#         self.declare_parameter("icp_voxel_size", 0.02)
#         self.declare_parameter("icp_max_correspondence_distance", 0.05)
#         self.declare_parameter("icp_downsample_voxel", 0.02)
#         self.declare_parameter("icp_run_interval", 1.0)  # seconds between ICP runs (if no new frames waits up to this)

#         # === NEW GTSAM / loop closure params ===
#         self.declare_parameter("enable_gtsam", True)
#         self.declare_parameter("gtsam_window_size", 200)  # how many nodes to keep in the optimization window
#         self.declare_parameter("gtsam_run_interval", 5.0)  # seconds between periodic optimizations
#         self.declare_parameter("gtsam_opt_interval_frames", 20)  # also trigger optimization every N frames
        
#         rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
#         depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
#         odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
#         self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

#         # read icp params
#         self.pcd_buffer_size = self.get_parameter("pcd_buffer_size").get_parameter_value().integer_value
#         self.icp_voxel_size = self.get_parameter("icp_voxel_size").get_parameter_value().double_value
#         self.icp_max_corr = self.get_parameter("icp_max_correspondence_distance").get_parameter_value().double_value
#         self.icp_downsample_voxel = self.get_parameter("icp_downsample_voxel").get_parameter_value().double_value
#         self.icp_run_interval = self.get_parameter("icp_run_interval").get_parameter_value().double_value

#         # gtsam params
#         self.enable_gtsam = self.get_parameter("enable_gtsam").get_parameter_value().bool_value and _HAS_GTSAM
#         if self.get_parameter("enable_gtsam").get_parameter_value().bool_value and not _HAS_GTSAM:
#             self.get_logger().warn("GTSAM Python bindings not available. GTSAM optimizer disabled (no-op fallback).")
#             self.enable_gtsam = False

#         self.gtsam_window_size = self.get_parameter("gtsam_window_size").get_parameter_value().integer_value
#         self.gtsam_run_interval = self.get_parameter("gtsam_run_interval").get_parameter_value().double_value
#         self.gtsam_opt_interval_frames = self.get_parameter("gtsam_opt_interval_frames").get_parameter_value().integer_value
        
#         # ---- Static extrinsic: body(imu) → cam0 ----
#         self.T_body_cam = np.array([
#             [-0.02156132, -0.99483675, -0.09917134,  0.03432751],
#             [-0.25782609,  0.10137213, -0.96085868,  0.03072772],
#             [ 0.96595073,  0.00485158, -0.25868058, -0.07285357],
#             [ 0.,          0.,          0.,          1.        ]
#         ])

#         # keep flip matrix accessible for gtsam thread
#         self.flip_rosopt_to_open3d = np.array([
#             [1, 0, 0, 0],
#             [0,-1, 0, 0],
#             [0, 0,-1, 0],
#             [0, 0, 0, 1]
#         ])

#         # ---- Intrinsics (replace with your calibrated values) ----
#         fx, fy, cx, cy = 525.0, 525.0, 319.5, 239.5
#         width, height = 640, 480
#         self.intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

#         # ---- TSDF volume ----
#         # self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
#         #     voxel_length=0.02,
#         #     sdf_trunc=0.04,
#         #     color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
#         # )

#         # ---- Subscribers ----
#         self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
#         self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
#         self.odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)

#         self.ts = message_filters.ApproximateTimeSynchronizer(
#             [self.rgb_sub, self.depth_sub, self.odom_sub],
#             queue_size=10, slop=0.01
#         )
#         self.ts.registerCallback(self.sync_callback)

#         # ---- Helpers ----
#         self.bridge = CvBridge()
#         self.frame_id = 0
#         self.pose_records = []
#         self.global_pcd = o3d.geometry.PointCloud()
#         self.depth_trunc = 2.0
#         self.min_valid_depth_pixels = 500  # skip frames with almost no depth

#         # === ICP buffer & threading ===
#         # buffer entries are dicts: {"pcd": o3d.geometry.PointCloud (camera frame), "pose": T_w_cam_open3d (world), "frame": id, "stamp": str}
#         self.pcd_buffer = []
#         self.buffer_lock = threading.Lock()
#         self.new_frame_event = threading.Event()
#         self.shutdown_event = threading.Event()
#         # start ICP thread
#         self.icp_thread = threading.Thread(target=self._icp_worker, daemon=True)
#         self.icp_thread.start()

#         # === NEW: GTSAM / graph bookkeeping ===
#         # graph_nodes: dict frame_id -> {'frame': int, 'odom_pose': 4x4 T_w_cam, 'pose_open3d': 4x4, 'stamp': str}
#         self.graph_nodes = {}
#         self.graph_nodes_order = []  # keep insertion order (frame ids)
#         self.opt_event = threading.Event()
#         # start gtsam thread (if enabled or even if disabled keep a thread for no-op behavior)
#         self.gtsam_thread = threading.Thread(target=self._gtsam_worker, daemon=True)
#         self.gtsam_thread.start()

#         self.get_logger().info("TSDF fusion node initialized (RGBD + odometry_rect). ICP & GTSAM workers started.")

#     def _depth_msg_to_numpy(self, depth_msg, depth_cv_raw):
#         """Convert depth cv2 image to numpy with correct dtype + scale."""
#         encoding = depth_msg.encoding.lower()
#         if encoding in ("16uc1", "mono16"):
#             depth_np = np.asarray(depth_cv_raw, dtype=np.uint16)
#             depth_scale = 1000.0
#         elif encoding in ("32fc1", "float32"):
#             depth_np = np.asarray(depth_cv_raw, dtype=np.float32)
#             depth_scale = 1.0
#         else:
#             # fallback: try uint16
#             depth_np = np.asarray(depth_cv_raw).astype(np.uint16)
#             depth_scale = 1000.0
#         return depth_np, depth_scale

#     def odom_to_matrix(self, odom: Odometry):
#         """Convert nav_msgs/Odometry pose to 4x4 transformation matrix."""
#         pose = odom.pose.pose
#         tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
#         qx, qy, qz, qw = (
#             pose.orientation.x,
#             pose.orientation.y,
#             pose.orientation.z,
#             pose.orientation.w,
#         )

#         norm = qw*qw + qx*qx + qy*qy + qz*qz
#         s = 1.0 / np.sqrt(norm)
#         qw, qx, qy, qz = qw*s, qx*s, qy*s, qz*s
#         R = np.array([
#             [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
#             [2*(qx*qy+qz*qw),     1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
#             [2*(qx*qz-qy*qw),     2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]
#         ], dtype=np.float64)
#         T = np.eye(4)
#         T[:3, :3] = R
#         T[:3, 3] = np.array([tx, ty, tz])
#         return T

#     def sync_callback(self, rgb_msg: Image, depth_msg: Image, odom_msg: Odometry):
#         # --- Convert RGB ---
#         try:
#             rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"RGB cv_bridge failed: {e}")
#             return

#         # --- Convert Depth (robust) ---
#         try:
#             depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Depth cv_bridge passthrough failed: {e}")
#             try:
#                 depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
#             except Exception as e2:
#                 self.get_logger().error(f"Depth cv_bridge fallback failed: {e2}")
#                 return

#         try:
#             depth_np, depth_scale = self._depth_msg_to_numpy(depth_msg, depth_cv_raw)
#         except Exception as e:
#             self.get_logger().error(f"Depth -> numpy sanitization failed: {e}")
#             return

#         valid = int(np.count_nonzero(depth_np))
#         if valid < self.min_valid_depth_pixels:
#             self.get_logger().warn(f"Skipping frame {self.frame_id}: too few valid depth pixels ({valid}).")
#             return

#         # resize RGB if mismatch
#         if rgb_cv.shape[0] != self.intrinsics.height or rgb_cv.shape[1] != self.intrinsics.width:
#             rgb_cv = cv2.resize(rgb_cv, (self.intrinsics.width, self.intrinsics.height))

#         # --- Build Open3D RGB + Depth ---
#         try:
#             color_o3d = o3d.geometry.Image(cv2.cvtColor(np.ascontiguousarray(rgb_cv), cv2.COLOR_BGR2RGB))
#             depth_o3d = o3d.geometry.Image(np.ascontiguousarray(depth_np))
#         except Exception as e:
#             self.get_logger().error(f"Open3D image creation failed: {e}")
#             return

#         try:
#             rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
#                 color_o3d, depth_o3d,
#                 depth_scale=depth_scale,
#                 depth_trunc=self.depth_trunc,
#                 convert_rgb_to_intensity=False
#             )
#         except Exception as e:
#             self.get_logger().error(f"RGBD creation failed: {e}")
#             return

#         # --- Get odometry pose ---
#         T_w_body = self.odom_to_matrix(odom_msg)
#         T_w_cam = T_w_body @ self.T_body_cam

#         # --- Fuse (note: keep pcd in camera frame, DO NOT apply T_w_cam here) ---
#         try:
#             pcd_cam = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)
#             # convert orientation convention but keep cloud in camera frame
#             pcd_cam.transform(self.flip_rosopt_to_open3d)
#             # compute corresponding world pose for this Open3D camera frame (so poses and clouds share convention)
#             pose_open3d = T_w_cam @ self.flip_rosopt_to_open3d
#         except Exception as e:
#             self.get_logger().warn(f"Open3D processing failed: {e}")
#             return

#         # --- Save pose record (odometry pose for now) ---
#         row = [self.frame_id, f"{rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec:09d}"]
#         row.extend(T_w_cam.reshape(-1).tolist())
#         with self.buffer_lock:
#             self.pose_records.append(row)

#         # === NEW: add downsampled copy of pcd_cam to buffer for ICP (store pcd in camera frame + pose) ===
#         try:
#             # create a copy / downsample to keep memory bounded
#             tmp = pcd_cam.voxel_down_sample(voxel_size=self.icp_downsample_voxel)
#             # open3d pointcloud has clone in recent versions; fallback to deepcopy
#             try:
#                 pcd_copy = tmp.clone()
#             except Exception:
#                 pcd_copy = deepcopy(tmp)

#             entry = {
#                 "pcd": pcd_copy,
#                 "pose": pose_open3d.copy(),  # initially from odometry
#                 "frame": self.frame_id,
#                 "stamp": f"{rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec:09d}"
#             }

#             with self.buffer_lock:
#                 self.pcd_buffer.append(entry)
#                 # keep buffer bounded: drop oldest if needed
#                 if len(self.pcd_buffer) > self.pcd_buffer_size:
#                     # drop oldest
#                     self.pcd_buffer.pop(0)

#                 # === NEW: add graph node bookkeeping (keep larger window than pcd_buffer) ===
#                 gn = {
#                     'frame': self.frame_id,
#                     'odom_pose': T_w_cam.copy(),
#                     'pose_open3d': pose_open3d.copy(),
#                     'stamp': entry['stamp']
#                 }
#                 self.graph_nodes[self.frame_id] = gn
#                 self.graph_nodes_order.append(self.frame_id)
#                 # trim graph_nodes to window size
#                 if len(self.graph_nodes_order) > self.gtsam_window_size:
#                     oldest = self.graph_nodes_order.pop(0)
#                     try:
#                         del self.graph_nodes[oldest]
#                     except Exception:
#                         pass

#             # signal ICP thread
#             self.new_frame_event.set()

#         except Exception as e:
#             self.get_logger().warn(f"Failed to push pcd to ICP buffer: {e}")

#         self.frame_id += 1


#     # === CLEAN: GTSAM optimization worker (odometry only, no loop closures) ===
#     def _gtsam_worker(self):
#         """
#         Background worker: when optimization is triggered (periodically),
#         build a small pose-graph from the last gtsam_window_size frames using only
#         odometry constraints, and run GTSAM optimization.
#         After optimization, update graph_nodes and any matching entries in pcd_buffer.
#         """
#         self.get_logger().info("GTSAM worker thread started. enabled=%s" % str(self.enable_gtsam))
#         last_opt_frame = -1

#         # helper converters
#         # helper converters (robust to different gtsam/python bindings)
#         def mat4_to_gtsam_pose(T):
#             """Convert numpy 4x4 to gtsam.Pose3 (robust constructors)."""
#             if not _HAS_GTSAM:
#                 return None
#             R = np.asarray(T[:3, :3], dtype=float)
#             t = np.asarray(T[:3, 3], dtype=float).reshape(3,)
#             try:
#                 return gtsam.Pose3(gtsam.Rot3(R), gtsam.Point3(float(t[0]), float(t[1]), float(t[2])))
#             except Exception:
#                 # Some gtsam bindings expose Rot3.Matrix or require different ctor
#                 try:
#                     return gtsam.Pose3(gtsam.Rot3.Matrix(R), gtsam.Point3(float(t[0]), float(t[1]), float(t[2])))
#                 except Exception as e:
#                     self.get_logger().warn(f"mat4_to_gtsam_pose: failed to build Pose3: {e}")
#                     raise

#         def _extract_point3_coords(t):
#             """Return (x,y,z) floats from a gtsam Point3-like or array-like object."""
#             # gtsam.Point3 usually has callable methods x(), y(), z()
#             try:
#                 if hasattr(t, "x"):
#                     attr = getattr(t, "x")
#                     if callable(attr):
#                         tx = float(attr())
#                         ty = float(getattr(t, "y")())
#                         tz = float(getattr(t, "z")())
#                         return tx, ty, tz
#                     else:
#                         # properties instead of callables
#                         tx = float(getattr(t, "x"))
#                         ty = float(getattr(t, "y"))
#                         tz = float(getattr(t, "z"))
#                         return tx, ty, tz
#             except Exception:
#                 pass

#             # array-like fallback (numpy array, list, tuple)
#             try:
#                 arr = np.asarray(t).flatten()
#                 return float(arr[0]), float(arr[1]), float(arr[2])
#             except Exception as e:
#                 # last resort: try to access indices/attributes explicitly
#                 try:
#                     return float(t[0]), float(t[1]), float(t[2])
#                 except Exception:
#                     raise RuntimeError(f"_extract_point3_coords: cannot extract coords from object of type {type(t)}: {e}")

#         def gtsam_pose_to_mat4(pose3):
#             """Convert gtsam.Pose3 (or compatible) to 4x4 numpy matrix robustly."""
#             if not _HAS_GTSAM:
#                 return None

#             # rotation -> numpy 3x3
#             try:
#                 R = pose3.rotation().matrix()
#             except Exception:
#                 # maybe rotation() already returned a numpy-like or different API
#                 try:
#                     R = np.asarray(pose3.rotation())
#                 except Exception as e:
#                     raise RuntimeError(f"gtsam_pose_to_mat4: cannot get rotation matrix from pose3: {e}")

#             R = np.asarray(R, dtype=float)
#             if R.shape != (3, 3):
#                 raise RuntimeError(f"gtsam_pose_to_mat4: unexpected rotation shape {R.shape}")

#             # translation -> (x,y,z)
#             try:
#                 t = pose3.translation()
#                 tx, ty, tz = _extract_point3_coords(t)
#             except Exception as e:
#                 # try result.atPose3 -> translation fallback (rare)
#                 try:
#                     arr = np.asarray(pose3)
#                     tx, ty, tz = float(arr[0]), float(arr[1]), float(arr[2])
#                 except Exception:
#                     raise RuntimeError(f"gtsam_pose_to_mat4: cannot extract translation from pose3: {e}")

#             T = np.eye(4, dtype=float)
#             T[:3, :3] = R
#             T[:3, 3] = np.array([tx, ty, tz], dtype=float)
#             return T


#         while not self.shutdown_event.is_set():
#             # wait for trigger or periodic interval
#             triggered = self.opt_event.wait(self.gtsam_run_interval)
#             self.opt_event.clear()
#             if self.shutdown_event.is_set():
#                 break

#             # snapshot graph_nodes
#             with self.buffer_lock:
#                 frames = list(self.graph_nodes_order)
#                 nodes = {f: deepcopy(self.graph_nodes[f]) for f in frames}

#             if len(nodes) == 0:
#                 continue

#             # take last window
#             frames_window = frames[-self.gtsam_window_size:]
#             if len(frames_window) == 0:
#                 continue

#             # optimization trigger guard
#             if frames_window[-1] == last_opt_frame and not triggered:
#                 continue
#             last_opt_frame = frames_window[-1]

#             try:
#                 if _HAS_GTSAM and self.enable_gtsam:
#                     graph = gtsam.NonlinearFactorGraph()
#                     initial = gtsam.Values()

#                     # noise models
#                     prior_sigma = np.array([1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2])
#                     odom_sigma  = np.array([0.05, 0.05, 0.05, 0.1, 0.1, 0.1])
#                     prior_noise = gtsam.noiseModel.Diagonal.Sigmas(prior_sigma)
#                     odom_noise  = gtsam.noiseModel.Diagonal.Sigmas(odom_sigma)

#                     # map frames to keys
#                     frame_to_idx = {f: idx for idx, f in enumerate(frames_window)}

#                     # add nodes
#                     for f, idx in frame_to_idx.items():
#                         T = nodes[f]['odom_pose']
#                         pose3 = mat4_to_gtsam_pose(T)
#                         key = gtsam.symbol('x', idx)
#                         initial.insert(key, pose3)

#                     # prior on first node
#                     first_key = gtsam.symbol('x', 0)
#                     graph.add(gtsam.PriorFactorPose3(first_key, initial.atPose3(first_key), prior_noise))

#                     # odometry edges between consecutive nodes
#                     for i in range(len(frames_window)-1):
#                         f_i, f_j = frames_window[i], frames_window[i+1]
#                         idx_i, idx_j = frame_to_idx[f_i], frame_to_idx[f_j]
#                         T_i, T_j = nodes[f_i]['odom_pose'], nodes[f_j]['odom_pose']
#                         T_rel = np.linalg.inv(T_i) @ T_j
#                         pose_rel = mat4_to_gtsam_pose(T_rel)
#                         graph.add(gtsam.BetweenFactorPose3(
#                             gtsam.symbol('x', idx_i),
#                             gtsam.symbol('x', idx_j),
#                             pose_rel, odom_noise
#                         ))

#                     # optimize
#                     params = gtsam.LevenbergMarquardtParams()
#                     params.setVerbosityLM("ERROR")
#                     optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
#                     result = optimizer.optimize()

#                     # update poses
#                     updated_count = 0
#                     with self.buffer_lock:
#                         for f, idx in frame_to_idx.items():
#                             key = gtsam.symbol('x', idx)
#                             if result.exists(key):
#                                 opt_pose = result.atPose3(key)
#                                 T_opt = gtsam_pose_to_mat4(opt_pose)
#                                 if f in self.graph_nodes:
#                                     self.graph_nodes[f]['odom_pose'] = T_opt.copy()
#                                     self.graph_nodes[f]['pose_open3d'] = T_opt @ self.flip_rosopt_to_open3d
#                                     updated_count += 1
#                                     # update pose_records if any
#                                     for rr in self.pose_records:
#                                         if rr[0] == f:
#                                             rr[2:] = T_opt.reshape(-1).tolist()
#                                             break
#                                     # update pcd_buffer
#                                     for e in self.pcd_buffer:
#                                         if e['frame'] == f:
#                                             e['pose'] = self.graph_nodes[f]['pose_open3d'].copy()

#                     self.get_logger().info(
#                         f"GTSAM optimization finished. updated {updated_count}/{len(frame_to_idx)} nodes (odometry-only)."
#                     )
#                 else:
#                     # fallback (no gtsam)
#                     with self.buffer_lock:
#                         updated_count = 0
#                         for f in frames_window:
#                             T = self.graph_nodes[f]['odom_pose']
#                             self.graph_nodes[f]['pose_open3d'] = T @ self.flip_rosopt_to_open3d
#                             for e in self.pcd_buffer:
#                                 if e['frame'] == f:
#                                     e['pose'] = self.graph_nodes[f]['pose_open3d'].copy()
#                                     updated_count += 1
#                     self.get_logger().info(f"GTSAM disabled - no-op applied. updated {updated_count} entries.")
#             except Exception as e:
#                 self.get_logger().warn(f"GTSAM worker failed: {e}")
#                 time.sleep(0.05)

#         self.get_logger().info("GTSAM worker thread exiting.")


#     # === NEW: ICP worker that registers the buffer into a merged cloud, then aligns to global_pcd ===
#     def _icp_worker(self):
#         """
#         Background worker: waits for new frames, then performs ICP on a snapshot of the buffer,
#         producing a merged point cloud (all in world/Open3D-camera coords). That merged chunk is
#         then aligned to the global_pcd using multi-scale ICP and inserted into global_pcd.
#         """
#         self.get_logger().info("ICP worker thread started.")
#         last_global_transform = np.eye(4)

#         loss_type = getattr(self, "icp_loss_type", "tukey").lower()
#         def make_loss(scale):
#             if loss_type == "tukey":
#                 return o3d.pipelines.registration.TukeyLoss(scale)
#             elif loss_type == "huber":
#                 return o3d.pipelines.registration.HuberLoss(scale)
#             else:
#                 return None

#         while not self.shutdown_event.is_set():
#             self.new_frame_event.wait(self.icp_run_interval)
#             self.new_frame_event.clear()
#             if self.shutdown_event.is_set():
#                 break

#             with self.buffer_lock:
#                 buffer_snapshot = [
#                     {
#                         "pcd": (e["pcd"].clone() if hasattr(e["pcd"], "clone") else deepcopy(e["pcd"])),
#                         "pose": e["pose"].copy(),
#                         "frame": e["frame"],
#                         "stamp": e["stamp"]
#                     }
#                     for e in self.pcd_buffer
#                 ]
#                 buffer_snapshot.sort(key=lambda x: x['frame'])

#             if len(buffer_snapshot) == 0:
#                 continue

#             try:
#                 # Reference (target) is the first frame in buffer_snapshot
#                 target_item = buffer_snapshot[0]
#                 target_pcd = target_item["pcd"].voxel_down_sample(self.icp_voxel_size)
#                 target_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30))
#                 target_pose = target_item["pose"].copy()   # world -> Open3D camera (fixed)
#                 target_frame = target_item.get("frame", -1)

#                 # Create merged_world seeded by transforming target into world
#                 merged_world = deepcopy(target_pcd)
#                 merged_world.transform(target_pose)   # now merged_world is in world coords

#                 # For each remaining source, register it to the TARGET (not to a drifting merged frame)
#                 for i in range(1, len(buffer_snapshot)):
#                     src_item = buffer_snapshot[i]
#                     source = src_item["pcd"]
#                     source_pose = src_item["pose"].copy()  # world -> Open3D camera

#                     # downsample and normals for registration in camera frames
#                     src_down = source.voxel_down_sample(self.icp_voxel_size)
#                     src_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30))

#                     # initial guess: map source_cam -> target_cam
#                     T_init = np.linalg.inv(target_pose) @ source_pose

#                     # debug: magnitude of initial transform
#                     try:
#                         t_mag = np.linalg.norm(T_init[:3,3])
#                         rmat = T_init[:3,:3]
#                         angle = np.arccos(np.clip((np.trace(rmat)-1)/2, -1.0, 1.0))
#                         self.get_logger().debug(f"T_init {src_item.get('frame')} -> {target_frame}: trans={t_mag:.4f} rot_deg={np.degrees(angle):.2f}")
#                     except Exception:
#                         pass

#                     # coarse: point-to-point in target frame
#                     reg_p2p = o3d.pipelines.registration.registration_icp(
#                         src_down, target_pcd,
#                         max_correspondence_distance=self.icp_max_corr * 4.0,
#                         init=T_init,
#                         estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
#                     )

#                     # refine: point-to-plane in target frame
#                     reg = o3d.pipelines.registration.registration_icp(
#                         src_down, target_pcd,
#                         self.icp_max_corr,
#                         reg_p2p.transformation,
#                         o3d.pipelines.registration.TransformationEstimationPointToPlane(make_loss(self.icp_voxel_size)),
#                         o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
#                     )

#                     self.get_logger().debug(f"pairwise reg frame {src_item['frame']} -> target {target_frame}: fitness={reg.fitness:.4f}, rmse={reg.inlier_rmse:.4f}")

#                     # transform source into target frame (reg.transformation maps source_cam -> target_cam)
#                     source_in_target = deepcopy(source)
#                     source_in_target.transform(reg.transformation)   # now in target_cam frame

#                     # then transform into world using target_pose (target_cam -> world)
#                     source_in_world = deepcopy(source_in_target)
#                     source_in_world.transform(target_pose)           # now in world coords

#                     # add to merged_world (world coords)
#                     merged_world += source_in_world

#                 # after all sources added, clean merged_world (voxel + normals)
#                 try:
#                     merged_world = merged_world.voxel_down_sample(self.icp_downsample_voxel)
#                     merged_world.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_downsample_voxel*2, max_nn=30))
#                 except Exception:
#                     pass

#                 # --- Step 2: align merged_world to global_pcd (if exists) using multi-scale ICP ---
#                 with self.buffer_lock:
#                     global_copy = None
#                     if len(self.global_pcd.points) > 0:
#                         global_copy = self.global_pcd.voxel_down_sample(self.icp_voxel_size)
#                         global_copy.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30))

#                 if global_copy is not None:
#                     voxel_radii = [self.icp_voxel_size*4, self.icp_voxel_size*2, self.icp_voxel_size]
#                     max_corrs = [self.icp_max_corr*4.0, self.icp_max_corr*2.0, self.icp_max_corr]
#                     current_trans = np.eye(4)   # merged_world already in world coords

#                     for idx, (scale, max_corr) in enumerate(zip(voxel_radii, max_corrs)):
#                         src_down = merged_world.voxel_down_sample(scale)
#                         tgt_down = global_copy.voxel_down_sample(scale)
#                         src_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=scale*2, max_nn=30))
#                         tgt_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=scale*2, max_nn=30))

#                         if idx == 0:
#                             reg = o3d.pipelines.registration.registration_icp(
#                                 src_down, tgt_down, max_corr, current_trans,
#                                 o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#                                 o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
#                             )
#                         else:
#                             reg = o3d.pipelines.registration.registration_icp(
#                                 src_down, tgt_down, max_corr, current_trans,
#                                 o3d.pipelines.registration.TransformationEstimationPointToPlane(make_loss(scale)),
#                                 o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
#                             )

#                         current_trans = reg.transformation
#                         self.get_logger().debug(f"global-scale {scale:.3f} fitness={reg.fitness:.4f} rmse={reg.inlier_rmse:.4f}")

#                     merged_world.transform(current_trans)

#                 # --- Step 3: merge into global map and clean duplicates/outliers ---
#                 with self.buffer_lock:
#                     if len(self.global_pcd.points) == 0:
#                         self.global_pcd = merged_world
#                     else:
#                         try:
#                             if _HAS_SCIPY:
#                                 glob_np = np.asarray(self.global_pcd.points)
#                                 merged_np = np.asarray(merged_world.points)
#                                 if glob_np.size == 0:
#                                     keep_idx = np.arange(len(merged_np))
#                                 else:
#                                     tree = cKDTree(glob_np)
#                                     dists, _ = tree.query(merged_np, k=1, n_jobs=-1)
#                                     thresh = max(self.icp_downsample_voxel * 0.5, 1e-3)
#                                     keep_idx = np.where(dists > thresh)[0]

#                                 if len(keep_idx) > 0:
#                                     new_pc = o3d.geometry.PointCloud()
#                                     new_pc.points = o3d.utility.Vector3dVector(merged_np[keep_idx])
#                                     try:
#                                         merged_colors = np.asarray(merged_world.colors)
#                                         new_pc.colors = o3d.utility.Vector3dVector(merged_colors[keep_idx])
#                                     except Exception:
#                                         pass
#                                     self.global_pcd += new_pc
#                             else:
#                                 self.global_pcd += merged_world

#                             # coarse voxel downsample then statistical outlier removal
#                             self.global_pcd = self.global_pcd.voxel_down_sample(self.icp_downsample_voxel)
#                             try:
#                                 filtered, ind = self.global_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
#                                 self.global_pcd = filtered
#                             except Exception:
#                                 pass

#                             try:
#                                 self.global_pcd.remove_duplicated_points()
#                             except Exception:
#                                 pass

#                         except Exception as e:
#                             try:
#                                 self.global_pcd += merged_world
#                                 self.global_pcd = self.global_pcd.voxel_down_sample(self.icp_downsample_voxel)
#                             except Exception:
#                                 pass

#                 self.get_logger().info(f"ICP worker: merged {len(buffer_snapshot)} -> global total {len(self.global_pcd.points)}")
#             except Exception as e:
#                 self.get_logger().warn(f"ICP worker failed: {e}")
#                 time.sleep(0.1)

#         self.get_logger().info("ICP worker thread exiting.")


#     def save_results(self):
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         out_dir = f"tsdf_output_{timestamp}"
#         os.makedirs(out_dir, exist_ok=True)

#         # mesh = self.volume.extract_triangle_mesh()
#         # mesh.compute_vertex_normals()
#         # o3d.io.write_triangle_mesh(os.path.join(out_dir, "mesh.ply"), mesh)

#         o3d.io.write_point_cloud(os.path.join(out_dir, "global_pcd.ply"), self.global_pcd)

#         with open(os.path.join(out_dir, "poses.csv"), "w") as f:
#             writer = csv.writer(f)
#             writer.writerow(["frame_id", "timestamp"] + [f"T{i}{j}" for i in range(4) for j in range(4)])
#             writer.writerows(self.pose_records)

#         self.get_logger().info(f"Saved results to {out_dir}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = TSDFFusionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # signal ICP & GTSAM workers to exit and wait for them
#         try:
#             node.shutdown_event.set()
#             node.new_frame_event.set()  # wake the ICP thread if it's waiting
#             node.opt_event.set()        # wake gtsam thread if waiting
#             if node.icp_thread.is_alive():
#                 node.get_logger().info("Waiting for ICP thread to finish...")
#                 node.icp_thread.join(timeout=5.0)
#             if node.gtsam_thread.is_alive():
#                 node.get_logger().info("Waiting for GTSAM thread to finish...")
#                 node.gtsam_thread.join(timeout=5.0)
#         except Exception:
#             pass

#         node.save_results()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import message_filters
# from sensor_msgs.msg import Image
# from nav_msgs.msg import Odometry
# from cv_bridge import CvBridge
# import numpy as np
# import open3d as o3d
# import cv2
# import csv
# import os
# from datetime import datetime

# # === NEW imports ===
# import threading
# import time
# from copy import deepcopy

# # GTSAM for pose graph optimization (optional)
# try:
#     import gtsam
#     from gtsam import Pose3, Rot3, Point3
#     from gtsam import noiseModel
#     GTSAM_AVAILABLE = True
# except Exception:
#     GTSAM_AVAILABLE = False

# class TSDFFusionNode(Node):
#     def __init__(self):
#         super().__init__('tsdf_fusion_with_odometry')

#         # ---- Parameters ----
#         self.declare_parameter("rgb_topic", "/ascamera_hp60c/camera_publisher/rgb0/image")
#         self.declare_parameter("depth_topic", "/ascamera_hp60c/camera_publisher/depth0/image_raw")
#         self.declare_parameter("odom_topic", "/odometry_rect")
#         self.declare_parameter("camera_frame", "cam0")
#         # new: buffer size and icp settings
#         self.declare_parameter("pcd_buffer_size", 10)  # number of recent frames to consider for ICP (was pcd buffer)
#         self.declare_parameter("icp_voxel_size", 0.02)
#         self.declare_parameter("icp_max_correspondence_distance", 0.05)
#         self.declare_parameter("icp_downsample_voxel", 0.02)
#         self.declare_parameter("icp_run_interval", 1.0)  # seconds between ICP runs (if no new frames waits up to this)
#         # pose graph params
#         self.declare_parameter("pose_graph_opt_interval", 20)  # run global optimize every N merges
#         self.declare_parameter("loop_search_stride", 10)  # test loop closure candidates every K nodes

#         rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
#         depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
#         odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
#         self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

#         # read icp params
#         self.pcd_buffer_size = self.get_parameter("pcd_buffer_size").get_parameter_value().integer_value
#         self.icp_voxel_size = self.get_parameter("icp_voxel_size").get_parameter_value().double_value
#         self.icp_max_corr = self.get_parameter("icp_max_correspondence_distance").get_parameter_value().double_value
#         self.icp_downsample_voxel = self.get_parameter("icp_downsample_voxel").get_parameter_value().double_value
#         self.icp_run_interval = self.get_parameter("icp_run_interval").get_parameter_value().double_value
#         self.pose_graph_opt_interval = self.get_parameter("pose_graph_opt_interval").get_parameter_value().integer_value
#         self.loop_search_stride = self.get_parameter("loop_search_stride").get_parameter_value().integer_value

#         # ---- Static extrinsic: body(imu) → cam0 ----
#         self.T_body_cam = np.array([
#             [-0.02156132, -0.99483675, -0.09917134,  0.03432751],
#             [-0.25782609,  0.10137213, -0.96085868,  0.03072772],
#             [ 0.96595073,  0.00485158, -0.25868058, -0.07285357],
#             [ 0.,          0.,          0.,          1.        ]
#         ])

#         # ---- Intrinsics (replace with your calibrated values) ----
#         fx, fy, cx, cy = 525.0, 525.0, 319.5, 239.5
#         width, height = 640, 480
#         self.intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

#         # ---- TSDF volume (kept for reintegration) ----
#         self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
#             voxel_length=0.02,
#             sdf_trunc=0.04,
#             color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
#         )

#         # ---- Subscribers ----
#         self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
#         self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
#         self.odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)

#         self.ts = message_filters.ApproximateTimeSynchronizer(
#             [self.rgb_sub, self.depth_sub, self.odom_sub],
#             queue_size=10, slop=0.01
#         )
#         self.ts.registerCallback(self.sync_callback)

#         # ---- Helpers ----
#         self.bridge = CvBridge()
#         self.frame_id = 0
#         self.pose_records = []
#         self.global_pcd = o3d.geometry.PointCloud()
#         self.depth_trunc = 2.0
#         self.min_valid_depth_pixels = 500  # skip frames with almost no depth

#         # === Now store RGBD frames buffer for reintegration instead of large pcd buffer ===
#         # store raw rgbd images and associated odom pose (to allow full TSDF re-integration later)
#         self.rgbd_buffer_keys = []            # circular buffer of recent frame keys (ints)
#         self.rgbd_frames = {}                 # key -> (color_o3d, depth_o3d, depth_scale)
#         self.rgbd_poses = {}                  # key -> T_w_cam

#         # === ICP/PGO threading primitives ===
#         self.buffer_lock = threading.Lock()
#         self.new_frame_event = threading.Event()
#         self.shutdown_event = threading.Event()
#         # start ICP thread
#         self.icp_thread = threading.Thread(target=self._icp_worker, daemon=True)
#         self.icp_thread.start()

#         # === Pose graph / storage for bundlefusion-style reintegration ===
#         if not GTSAM_AVAILABLE:
#             self.get_logger().warn("GTSAM not available: pose graph optimization disabled. Install gtsam for full functionality.")
#         self.pose_graph_nodes = []  # store keys (ints)
#         self.pose_graph_clouds = {}  # key -> merged cloud (o3d.geometry.PointCloud)
#         self.pose_graph_odom = {}  # key -> odom pose guess (4x4)

#         # gtsam structures
#         if GTSAM_AVAILABLE:
#             self.graph = gtsam.NonlinearFactorGraph()
#             self.initial_estimates = gtsam.Values()
#             self.noise_prior = noiseModel.Diagonal.Sigmas(np.array([1e-6,1e-6,1e-6, 1e-3,1e-3,1e-3]))
#             self.noise_odom = noiseModel.Diagonal.Sigmas(np.array([1e-2,1e-2,1e-2, 1e-1,1e-1,1e-1]))
#             self.noise_icp = noiseModel.Diagonal.Sigmas(np.array([5e-2,5e-2,5e-2, 1e-1,1e-1,1e-1]))

#         self.merge_count = 0

#         self.get_logger().info("TSDF fusion node initialized (RGBD + odometry_rect). ICP & PGO ready.")

#     def _depth_msg_to_numpy(self, depth_msg, depth_cv_raw):
#         """Convert depth cv2 image to numpy with correct dtype + scale."""
#         encoding = depth_msg.encoding.lower()
#         if encoding in ("16uc1", "mono16"):
#             depth_np = np.asarray(depth_cv_raw, dtype=np.uint16)
#             depth_scale = 1000.0
#         elif encoding in ("32fc1", "float32"):
#             depth_np = np.asarray(depth_cv_raw, dtype=np.float32)
#             depth_scale = 1.0
#         else:
#             # fallback: try uint16
#             depth_np = np.asarray(depth_cv_raw).astype(np.uint16)
#             depth_scale = 1000.0
#         return depth_np, depth_scale

#     def odom_to_matrix(self, odom: Odometry):
#         """Convert nav_msgs/Odometry pose to 4x4 transformation matrix."""
#         pose = odom.pose.pose
#         tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
#         qx, qy, qz, qw = (
#             pose.orientation.x,
#             pose.orientation.y,
#             pose.orientation.z,
#             pose.orientation.w,
#         )

#         norm = qw*qw + qx*qx + qy*qy + qz*qz
#         s = 1.0 / np.sqrt(norm)
#         qw, qx, qy, qz = qw*s, qx*s, qy*s, qz*s
#         R = np.array([
#             [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
#             [2*(qx*qy+qz*qw),     1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
#             [2*(qx*qz-qy*qw),     2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]
#         ], dtype=np.float64)
#         T = np.eye(4)
#         T[:3, :3] = R
#         T[:3, 3] = np.array([tx, ty, tz])
#         return T

#     def sync_callback(self, rgb_msg: Image, depth_msg: Image, odom_msg: Odometry):
#         # --- Convert RGB ---
#         try:
#             rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"RGB cv_bridge failed: {e}")
#             return

#         # --- Convert Depth (robust) ---
#         try:
#             depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Depth cv_bridge passthrough failed: {e}")
#             try:
#                 depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
#             except Exception as e2:
#                 self.get_logger().error(f"Depth cv_bridge fallback failed: {e2}")
#                 return

#         try:
#             depth_np, depth_scale = self._depth_msg_to_numpy(depth_msg, depth_cv_raw)
#         except Exception as e:
#             self.get_logger().error(f"Depth -> numpy sanitization failed: {e}")
#             return

#         valid = int(np.count_nonzero(depth_np))
#         if valid < self.min_valid_depth_pixels:
#             self.get_logger().warn(f"Skipping frame {self.frame_id}: too few valid depth pixels ({valid}).")
#             return
        
#         # resize RGB if mismatch
#         if rgb_cv.shape[0] != self.intrinsics.height or rgb_cv.shape[1] != self.intrinsics.width:
#             rgb_cv = cv2.resize(rgb_cv, (self.intrinsics.width, self.intrinsics.height))

#         # --- Build Open3D RGB + Depth ---
#         try:
#             color_o3d = o3d.geometry.Image(cv2.cvtColor(np.ascontiguousarray(rgb_cv), cv2.COLOR_BGR2RGB))
#             depth_o3d = o3d.geometry.Image(np.ascontiguousarray(depth_np))
#         except Exception as e:
#             self.get_logger().error(f"Open3D image creation failed: {e}")
#             return

#         try:
#             rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
#                 color_o3d, depth_o3d,
#                 depth_scale=depth_scale,
#                 depth_trunc=self.depth_trunc,
#                 convert_rgb_to_intensity=False
#             )
#         except Exception as e:
#             self.get_logger().error(f"RGBD creation failed: {e}")
#             return

#         # --- Get odometry pose ---
#         T_w_body = self.odom_to_matrix(odom_msg)
#         T_w_cam = T_w_body @ self.T_body_cam

#         # --- Fuse (integrate into TSDF live) ---
#         try:
#             pcd_cam = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)
#             flip_rosopt_to_open3d = np.array([
#                 [1, 0, 0, 0],
#                 [0,-1, 0, 0],
#                 [0, 0,-1, 0],
#                 [0, 0, 0, 1]
#             ])
#             pcd_cam.transform(flip_rosopt_to_open3d)
#             pcd_cam.transform(T_w_cam)
#             # don't append to global_pcd here; ICP worker manages global accumulation
#         except Exception as e:
#             self.get_logger().warn(f"Open3D processing failed: {e}")

#         # # Integrate into TSDF immediately (live integration)
#         try:
#             extrinsic = np.linalg.inv(T_w_cam)
#             self.volume.integrate(rgbd, self.intrinsics, extrinsic)
#         except Exception as e:
#             self.get_logger().warn(f"TSDF integrate failed: {e}")

#         # --- Save pose record & store RGBD for later reintegration ---
#         row = [self.frame_id, f"{rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec:09d}"]
#         row.extend(T_w_cam.reshape(-1).tolist())
#         self.pose_records.append(row)

#         # store raw rgbd + pose in buffers (for reintegration)
#         key = self.frame_id
#         with self.buffer_lock:
#             self.rgbd_frames[key] = (color_o3d, depth_o3d, depth_scale)
#             self.rgbd_poses[key] = T_w_cam
#             self.rgbd_buffer_keys.append(key)
#             # keep buffer bounded
#             if len(self.rgbd_buffer_keys) > self.pcd_buffer_size * 10:  # allow larger backlog for reintegration
#                 old = self.rgbd_buffer_keys.pop(0)
#                 try:
#                     del self.rgbd_frames[old]
#                     del self.rgbd_poses[old]
#                 except Exception:
#                     pass

#         # add to pose graph bookkeeping
#         self.pose_graph_nodes.append(key)
#         self.pose_graph_clouds[key] = deepcopy(pcd_cam)
#         self.pose_graph_odom[key] = T_w_cam

#         # notify ICP worker (it will create simple relative ICP factors)
#         self.new_frame_event.set()
#         self.frame_id += 1


#     def _prepare_downsampled_with_normals(self, pc: o3d.geometry.PointCloud, voxel: float):
#         """
#         Downsample a pointcloud and ensure normals are estimated.
#         Returns (pc_down, has_normals).
#         """
#         pc_down = pc.voxel_down_sample(voxel)
#         if len(pc_down.points) == 0:
#             return pc_down, False
#         # choose radius relative to voxel (avoid too small values)
#         radius = max(voxel * 2.0, 0.01)
#         try:
#             pc_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))
#         except Exception:
#             pass
#         has_normals = (hasattr(pc_down, "normals") and len(pc_down.normals) == len(pc_down.points) and len(pc_down.normals) > 0)
#         return pc_down, has_normals


#     def _icp_worker(self):
#         """
#         ICP worker:
#         - builds a merged cloud from recent RGBD frames (no heavy local-ICP)
#         - aligns merged -> global via multi-scale ICP
#         - appends to global_pcd
#         - creates simplified ICP factors for the pose graph (prev-node and loop checks)
#         - runs GTSAM optimization periodically and reintegrates TSDF with optimized poses
#         """
#         self.get_logger().info("ICP worker thread started.")
#         last_global_transform = np.eye(4)

#         # loss helper
#         loss_type = getattr(self, "icp_loss_type", "tukey").lower()

#         def make_loss(scale):
#             if loss_type == "tukey":
#                 return o3d.pipelines.registration.TukeyLoss(scale)
#             elif loss_type == "huber":
#                 return o3d.pipelines.registration.HuberLoss(scale)
#             else:
#                 return None

#         while not self.shutdown_event.is_set():
#             self.new_frame_event.wait(self.icp_run_interval)
#             self.new_frame_event.clear()
#             if self.shutdown_event.is_set():
#                 break

#             # sliding window of latest frame keys
#             with self.buffer_lock:
#                 keys = list(self.rgbd_buffer_keys[-self.pcd_buffer_size:])

#             if len(keys) == 0:
#                 continue

#             try:
#                 # -------------------------
#                 # Build merged cloud quickly (concatenate recent RGBD frames using stored poses)
#                 # -------------------------
#                 merged = o3d.geometry.PointCloud()
#                 for k in keys:
#                     data = self.rgbd_frames.get(k)
#                     if data is None:
#                         continue
#                     color_o3d, depth_o3d, depth_scale = data
#                     try:
#                         rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
#                             color_o3d, depth_o3d, depth_scale=depth_scale,
#                             depth_trunc=self.depth_trunc, convert_rgb_to_intensity=False)
#                         pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)
#                         flip = np.array([[1, 0, 0, 0],
#                                         [0, -1, 0, 0],
#                                         [0, 0, -1, 0],
#                                         [0, 0, 0, 1]])
#                         pcd.transform(flip)
#                         T_w_cam = self.rgbd_poses.get(k, np.eye(4))
#                         pcd.transform(T_w_cam)
#                         merged += pcd
#                     except Exception:
#                         # skip frames that fail
#                         continue

#                 # compact merged for ICP
#                 merged = merged.voxel_down_sample(self.icp_downsample_voxel)
#                 merged.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_downsample_voxel * 2, max_nn=30))

#                 # -------------------------
#                 # Prepare downsampled global copy for ICP (if exists)
#                 # -------------------------
#                 with self.buffer_lock:
#                     global_copy = None
#                     if len(self.global_pcd.points) > 0:
#                         # keep a copy for alignment
#                         global_copy = self.global_pcd.voxel_down_sample(self.icp_voxel_size)
#                         try:
#                             global_copy.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size * 2, max_nn=30))
#                         except Exception:
#                             pass

#                 # -------------------------
#                 # Multi-scale ICP: merged -> global_copy
#                 # -------------------------
#                 if global_copy is not None and len(global_copy.points) > 0 and len(merged.points) > 0:
#                     voxel_radii = [self.icp_voxel_size * 4, self.icp_voxel_size * 2, self.icp_voxel_size]
#                     max_iters = [50, 30, 14]
#                     current_trans = last_global_transform

#                     for scale, max_iter in zip(voxel_radii, max_iters):
#                         # prepare downsampled source/target with normals (helper ensures normals)
#                         src_down, src_has_normals = self._prepare_downsampled_with_normals(merged, scale)
#                         tgt_down, tgt_has_normals = self._prepare_downsampled_with_normals(global_copy, scale)

#                         # choose estimation method: prefer point-to-plane if target has normals
#                         if tgt_has_normals:
#                             estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane(make_loss(scale))
#                         else:
#                             estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()

#                         # run ICP with fallback
#                         try:
#                             reg = o3d.pipelines.registration.registration_icp(
#                                 src_down, tgt_down,
#                                 self.icp_max_corr * 10,  # coarse correspondence bound
#                                 current_trans,
#                                 estimation,
#                                 o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
#                             )
#                         except Exception as ex:
#                             # fallback to point-to-point with identity init if something goes wrong
#                             try:
#                                 reg = o3d.pipelines.registration.registration_icp(
#                                     src_down, tgt_down,
#                                     self.icp_max_corr * 10,
#                                     current_trans,
#                                     o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#                                     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
#                                 )
#                             except Exception as ex2:
#                                 self.get_logger().warn(f"ICP inner failed at scale {scale}: {ex2}")
#                                 break

#                         # update transform
#                         current_trans = reg.transformation
#                         # optional logging of fitness / rmse
#                         # self.get_logger().debug(f"ICP scale {scale}: fitness={reg.fitness:.4f}, rmse={reg.inlier_rmse:.4f}")

#                     last_global_transform = current_trans
#                     try:
#                         merged.transform(current_trans)
#                     except Exception:
#                         pass

#                 # -------------------------
#                 # Append merged to global_pcd (protected)
#                 # -------------------------
#                 with self.buffer_lock:
#                     if len(self.global_pcd.points) == 0:
#                         self.global_pcd = merged
#                     else:
#                         self.global_pcd += merged
#                         try:
#                             self.global_pcd = self.global_pcd.voxel_down_sample(self.icp_downsample_voxel)
#                         except Exception:
#                             pass

#                 # -------------------------
#                 # PGO bookkeeping (simplified relative factors)
#                 # -------------------------
#                 if len(keys) == 0:
#                     continue
#                 node_key = keys[-1]

#                 # register node if new
#                 if node_key not in self.pose_graph_nodes:
#                     self.pose_graph_nodes.append(node_key)
#                     self.pose_graph_clouds[node_key] = deepcopy(merged)
#                     self.pose_graph_odom[node_key] = self.rgbd_poses.get(node_key, np.eye(4))

#                 # GTSAM: add initial estimate & prior if missing, add sparse between factor to previous node
#                 if GTSAM_AVAILABLE:
#                     try:
#                         sym = gtsam.Symbol('x', node_key)
#                         # insert initial estimate if missing
#                         if not self.initial_estimates.exists(sym.key()):
#                             odom_guess = self.pose_graph_odom.get(node_key, np.eye(4))
#                             p3 = Pose3(Rot3(odom_guess[:3, :3]), Point3(odom_guess[0, 3], odom_guess[1, 3], odom_guess[2, 3]))
#                             self.initial_estimates.insert(sym.key(), p3)
#                             if node_key == 0:
#                                 # PRIOR must use integer key, not Symbol
#                                 self.graph.add(gtsam.PriorFactorPose3(node_key, p3, self.noise_prior))
#                     except Exception as e:
#                         self.get_logger().warn(f"GTSAM initial estimate insert failed for node {node_key}: {e}")

#                     # simplified ICP edge to previous node
#                     if len(self.pose_graph_nodes) > 1:
#                         prev = self.pose_graph_nodes[-2]
#                         prev_cloud = self.pose_graph_clouds.get(prev)
#                         if prev_cloud is not None and len(prev_cloud.points) > 0 and len(merged.points) > 0:
#                             src_down, _ = self._prepare_downsampled_with_normals(merged, self.icp_voxel_size)
#                             prev_down, prev_has_normals = self._prepare_downsampled_with_normals(prev_cloud, self.icp_voxel_size)

#                             estimation = (
#                                 o3d.pipelines.registration.TransformationEstimationPointToPlane()
#                                 if prev_has_normals else
#                                 o3d.pipelines.registration.TransformationEstimationPointToPoint()
#                             )

#                             try:
#                                 reg = o3d.pipelines.registration.registration_icp(
#                                     src_down, prev_down,
#                                     self.icp_max_corr,
#                                     np.eye(4),
#                                     estimation
#                                 )
#                                 if reg.fitness > 0.5 and reg.inlier_rmse < 0.25:
#                                     rel = reg.transformation
#                                     rel_pose = Pose3(Rot3(rel[:3, :3]), Point3(rel[0, 3], rel[1, 3], rel[2, 3]))
#                                     # BETWEEN factors: use integer keys
#                                     self.graph.add(gtsam.BetweenFactorPose3(prev, node_key, rel_pose, self.noise_icp))
#                             except Exception:
#                                 pass

#                     # sparse loop closures (integer keys only)
#                     for k in range(0, max(0, node_key - 50), self.loop_search_stride):
#                         if k in self.pose_graph_clouds and k != node_key and abs(node_key - k) > 5:
#                             candidate = self.pose_graph_clouds[k]
#                             if len(candidate.points) == 0:
#                                 continue
#                             src_down, _ = self._prepare_downsampled_with_normals(merged, self.icp_voxel_size * 2)
#                             cand_down, cand_has_normals = self._prepare_downsampled_with_normals(candidate, self.icp_voxel_size * 2)
#                             if len(src_down.points) == 0 or len(cand_down.points) == 0:
#                                 continue
#                             estimation = (
#                                 o3d.pipelines.registration.TransformationEstimationPointToPlane()
#                                 if cand_has_normals else
#                                 o3d.pipelines.registration.TransformationEstimationPointToPoint()
#                             )
#                             try:
#                                 reg = o3d.pipelines.registration.registration_icp(
#                                     src_down, cand_down,
#                                     self.icp_max_corr * 2,
#                                     np.eye(4),
#                                     estimation
#                                 )
#                                 if reg.fitness > 0.35 and reg.inlier_rmse < 0.4:
#                                     rel = reg.transformation
#                                     rel_pose = Pose3(Rot3(rel[:3, :3]), Point3(rel[0, 3], rel[1, 3], rel[2, 3]))
#                                     self.graph.add(gtsam.BetweenFactorPose3(k, node_key, rel_pose, self.noise_icp))
#                                     self.get_logger().info(f"Loop closure added between {k} and {node_key} (fitness {reg.fitness:.3f})")
#                             except Exception:
#                                 pass


#                 # increment merge count and maybe run GTSAM optimize + reintegrate
#                 self.merge_count += 1
#                 if GTSAM_AVAILABLE and (self.merge_count % self.pose_graph_opt_interval == 0):
#                     self.get_logger().info("Running GTSAM Pose Graph Optimization...")
#                     try:
#                         params = gtsam.LevenbergMarquardtParams()
#                         optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial_estimates, params)
#                         result = optimizer.optimize()
#                         # apply optimized poses and reintegrate TSDF
#                         self._apply_optimized_poses_and_reintegrate(result)
#                         self.get_logger().info("Pose graph optimization + TSDF reintegration complete.")
#                     except Exception as e:
#                         self.get_logger().warn(f"PGO failed: {e}")

#                 self.get_logger().info(f"ICP worker: processed keys {keys} -> global size {len(self.global_pcd.points)}")

#             except Exception as e:
#                 self.get_logger().warn(f"ICP worker failed: {e}")
#                 time.sleep(0.1)

#         self.get_logger().info("ICP worker thread exiting.")


#     def _apply_optimized_poses_and_reintegrate(self, gtsam_result):
#         """
#         Extract optimized poses from gtsam result, replace pose records, and rebuild TSDF from stored RGBD frames.
#         This version uses stored RGBD frames for accurate TSDF reintegration (BundleFusion-style).
#         """
#         self.get_logger().info("Applying optimized poses and rebuilding TSDF...")
#         # extract poses
#         optimized_poses = {}
#         for key in self.pose_graph_nodes:
#             try:
#                 p = gtsam_result.atPose3(gtsam.Symbol('x', key))
#                 R = p.rotation().matrix()
#                 t = p.translation()
#                 T = np.eye(4)
#                 T[:3,:3] = R
#                 T[:3,3] = np.array([t.x(), t.y(), t.z()])
#                 optimized_poses[key] = T
#             except Exception:
#                 # missing in result, fallback to odom
#                 optimized_poses[key] = self.pose_graph_odom.get(key, self.rgbd_poses.get(key, np.eye(4)))

#         # Recreate TSDF volume and re-integrate all rgbd frames using optimized poses
#         new_volume = o3d.pipelines.integration.ScalableTSDFVolume(
#             voxel_length=0.02,
#             sdf_trunc=0.04,
#             color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
#         )

#         new_global = o3d.geometry.PointCloud()
#         # iterate through stored frames (pose_graph_nodes order)
#         for key in self.pose_graph_nodes:
#             if key not in optimized_poses:
#                 continue
#             T_w_cam = optimized_poses[key]
#             # retrieve stored rgbd
#             data = self.rgbd_frames.get(key)
#             if data is None:
#                 continue
#             color_o3d, depth_o3d, depth_scale = data
#             try:
#                 rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d, depth_scale=depth_scale, depth_trunc=self.depth_trunc, convert_rgb_to_intensity=False)
#                 # integrate using optimized extrinsic
#                 extrinsic = np.linalg.inv(T_w_cam@flip_rosopt_to_open3d)
#                 new_volume.integrate(rgbd, self.intrinsics, extrinsic)
#                 # also build a global pointcloud for quick inspection
#                 pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)
#                 flip_rosopt_to_open3d = np.array([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
#                 pcd.transform(flip_rosopt_to_open3d)
#                 pcd.transform(T_w_cam)
#                 new_global += pcd
#             except Exception as e:
#                 self.get_logger().warn(f"Reintegration failed for key {key}: {e}")

#         try:
#             self.global_pcd = new_global.voxel_down_sample(self.icp_downsample_voxel)
#         except Exception:
#             self.global_pcd = new_global
#         self.volume = new_volume
#         # update pose_records with optimized poses
#         for i, key in enumerate(self.pose_graph_nodes):
#             if key in optimized_poses:
#                 T = optimized_poses[key]
#                 row = [key, 'opt']  # keep 'opt' marker instead of timestamp
#                 row.extend(T.reshape(-1).tolist())
#                 # replace if existing, otherwise append
#                 replaced = False
#                 for ridx, r in enumerate(self.pose_records):
#                     if r[0] == key:
#                         self.pose_records[ridx] = row
#                         replaced = True
#                         break
#                 if not replaced:
#                     self.pose_records.append(row)


#     def save_results(self):
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         out_dir = f"tsdf_output_{timestamp}"
#         os.makedirs(out_dir, exist_ok=True)

#         # write mesh from TSDF
#         try:
#             mesh = self.volume.extract_triangle_mesh()
#             mesh.compute_vertex_normals()
#             o3d.io.write_triangle_mesh(os.path.join(out_dir, "mesh.ply"), mesh)
#         except Exception as e:
#             self.get_logger().warn(f"Mesh export failed: {e}")

#         o3d.io.write_point_cloud(os.path.join(out_dir, "global_pcd.ply"), self.global_pcd)

#         with open(os.path.join(out_dir, "poses.csv"), "w") as f:
#             writer = csv.writer(f)
#             writer.writerow(["frame_id", "timestamp"] + [f"T{i}{j}" for i in range(4) for j in range(4)])
#             writer.writerows(self.pose_records)

#         self.get_logger().info(f"Saved results to {out_dir}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = TSDFFusionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # signal ICP worker to exit and wait for it
#         try:
#             node.shutdown_event.set()
#             node.new_frame_event.set()  # wake the thread if it's waiting
#             if node.icp_thread.is_alive():
#                 node.get_logger().info("Waiting for ICP thread to finish...")
#                 node.icp_thread.join(timeout=5.0)
#         except Exception:
#             pass

#         node.save_results()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()


# import rclpy
# from rclpy.node import Node
# import message_filters
# from sensor_msgs.msg import Image
# from nav_msgs.msg import Odometry
# from cv_bridge import CvBridge
# import numpy as np
# import open3d as o3d
# import cv2
# import csv
# import os
# from datetime import datetime

# # === NEW imports ===
# import threading
# import time
# from copy import deepcopy

# class TSDFFusionNode(Node):
#     def __init__(self):
#         super().__init__('tsdf_fusion_with_odometry')

#         # ---- Parameters ----
#         self.declare_parameter("rgb_topic", "/ascamera_hp60c/camera_publisher/rgb0/image")
#         self.declare_parameter("depth_topic", "/ascamera_hp60c/camera_publisher/depth0/image_raw")
#         self.declare_parameter("odom_topic", "/odometry_rect")
#         self.declare_parameter("camera_frame", "cam0")
#         # new: buffer size and icp settings
#         self.declare_parameter("pcd_buffer_size", 10)  # number of recent pcds to keep for ICP
#         self.declare_parameter("icp_voxel_size", 0.02)
#         self.declare_parameter("icp_max_correspondence_distance", 0.05)
#         self.declare_parameter("icp_downsample_voxel", 0.02)
#         self.declare_parameter("icp_run_interval", 1.0)  # seconds between ICP runs (if no new frames waits up to this)

#         rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
#         depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
#         odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
#         self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

#         # read icp params
#         self.pcd_buffer_size = self.get_parameter("pcd_buffer_size").get_parameter_value().integer_value
#         self.icp_voxel_size = self.get_parameter("icp_voxel_size").get_parameter_value().double_value
#         self.icp_max_corr = self.get_parameter("icp_max_correspondence_distance").get_parameter_value().double_value
#         self.icp_downsample_voxel = self.get_parameter("icp_downsample_voxel").get_parameter_value().double_value
#         self.icp_run_interval = self.get_parameter("icp_run_interval").get_parameter_value().double_value

#         # ---- Static extrinsic: body(imu) → cam0 ----
#         self.T_body_cam = np.array([
#             [-0.02156132, -0.99483675, -0.09917134,  0.03432751],
#             [-0.25782609,  0.10137213, -0.96085868,  0.03072772],
#             [ 0.96595073,  0.00485158, -0.25868058, -0.07285357],
#             [ 0.,          0.,          0.,          1.        ]
#         ])

#         # ---- Intrinsics (replace with your calibrated values) ----
#         fx, fy, cx, cy = 525.0, 525.0, 319.5, 239.5
#         width, height = 640, 480
#         self.intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

#         # ---- TSDF volume ----
#         # self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
#         #     voxel_length=0.02,
#         #     sdf_trunc=0.04,
#         #     color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
#         # )

#         # ---- Subscribers ----
#         self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
#         self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
#         self.odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)

#         self.ts = message_filters.ApproximateTimeSynchronizer(
#             [self.rgb_sub, self.depth_sub, self.odom_sub],
#             queue_size=10, slop=0.01
#         )
#         self.ts.registerCallback(self.sync_callback)

#         # ---- Helpers ----
#         self.bridge = CvBridge()
#         self.frame_id = 0
#         self.pose_records = []
#         self.global_pcd = o3d.geometry.PointCloud()
#         self.depth_trunc = 2.0
#         self.min_valid_depth_pixels = 500  # skip frames with almost no depth

#         # === ICP buffer & threading ===
#         # self.pcd_buffer = []  # list of recent pointclouds (o3d.geometry.PointCloud)
#         # self.buffer_lock = threading.Lock()
#         # self.new_frame_event = threading.Event()
#         # self.shutdown_event = threading.Event()
#         # # start ICP thread
#         # self.icp_thread = threading.Thread(target=self._icp_worker, daemon=True)
#         # self.icp_thread.start()

#                 # === ICP buffer & threading ===
#         self.pcd_buffer = []  # list of recent pointclouds (o3d.geometry.PointCloud)
#         self.buffer_lock = threading.Lock()
#         self.new_frame_event = threading.Event()
#         self.shutdown_event = threading.Event()

#         # --- NEW: merged submap buffer for two-stage pipeline ---
#         self.merged_buffer = []           # list of merged local submaps (o3d.geometry.PointCloud)
#         self.merged_lock = threading.Lock()
#         self.new_merged_event = threading.Event()

#         # --- Hybrid submap parameters (tunable) ---
#         self.declare_parameter("icp_submap_size", 10)   # N frames per submap
#         self.declare_parameter("icp_submap_time", 2.0)  # T seconds max per submap flush

#         self.icp_submap_size = self.get_parameter("icp_submap_size").get_parameter_value().integer_value
#         self.icp_submap_time = self.get_parameter("icp_submap_time").get_parameter_value().double_value

#         # track last submap push time to support the hybrid (N or T) rule
#         self._last_submap_time = time.time()

#         # start ICP threads: local (producer) + global (consumer)
#         self.local_thread = threading.Thread(target=self._local_merge_worker, daemon=True)
#         self.global_thread = threading.Thread(target=self._global_align_worker, daemon=True)
#         # keep icp_thread attribute for backwards compatibility with external shutdown code
#         self.icp_thread = self.local_thread
#         self.local_thread.start()
#         self.global_thread.start()

#         self.get_logger().info("TSDF fusion node initialized (RGBD + odometry_rect). ICP worker started.")

#     def _depth_msg_to_numpy(self, depth_msg, depth_cv_raw):
#         """Convert depth cv2 image to numpy with correct dtype + scale."""
#         encoding = depth_msg.encoding.lower()
#         if encoding in ("16uc1", "mono16"):
#             depth_np = np.asarray(depth_cv_raw, dtype=np.uint16)
#             depth_scale = 1000.0
#         elif encoding in ("32fc1", "float32"):
#             depth_np = np.asarray(depth_cv_raw, dtype=np.float32)
#             depth_scale = 1.0
#         else:
#             # fallback: try uint16
#             depth_np = np.asarray(depth_cv_raw).astype(np.uint16)
#             depth_scale = 1000.0
#         return depth_np, depth_scale

#     def odom_to_matrix(self, odom: Odometry):
#         """Convert nav_msgs/Odometry pose to 4x4 transformation matrix."""
#         pose = odom.pose.pose
#         tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
#         qx, qy, qz, qw = (
#             pose.orientation.x,
#             pose.orientation.y,
#             pose.orientation.z,
#             pose.orientation.w,
#         )

#         norm = qw*qw + qx*qx + qy*qy + qz*qz
#         s = 1.0 / np.sqrt(norm)
#         qw, qx, qy, qz = qw*s, qx*s, qy*s, qz*s
#         R = np.array([
#             [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
#             [2*(qx*qy+qz*qw),     1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
#             [2*(qx*qz-qy*qw),     2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]
#         ], dtype=np.float64)
#         T = np.eye(4)
#         T[:3, :3] = R
#         T[:3, 3] = np.array([tx, ty, tz])
#         return T

#     def sync_callback(self, rgb_msg: Image, depth_msg: Image, odom_msg: Odometry):
#         # --- Convert RGB ---
#         try:
#             rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"RGB cv_bridge failed: {e}")
#             return

#         # --- Convert Depth (robust) ---
#         try:
#             depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Depth cv_bridge passthrough failed: {e}")
#             try:
#                 depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
#             except Exception as e2:
#                 self.get_logger().error(f"Depth cv_bridge fallback failed: {e2}")
#                 return

#         try:
#             depth_np, depth_scale = self._depth_msg_to_numpy(depth_msg, depth_cv_raw)
#         except Exception as e:
#             self.get_logger().error(f"Depth -> numpy sanitization failed: {e}")
#             return

#         valid = int(np.count_nonzero(depth_np))
#         if valid < self.min_valid_depth_pixels:
#             self.get_logger().warn(f"Skipping frame {self.frame_id}: too few valid depth pixels ({valid}).")
#             return
        
#         # resize RGB if mismatch
#         if rgb_cv.shape[0] != self.intrinsics.height or rgb_cv.shape[1] != self.intrinsics.width:
#             rgb_cv = cv2.resize(rgb_cv, (self.intrinsics.width, self.intrinsics.height))

#         # --- Build Open3D RGB + Depth ---
#         try:
#             color_o3d = o3d.geometry.Image(cv2.cvtColor(np.ascontiguousarray(rgb_cv), cv2.COLOR_BGR2RGB))
#             depth_o3d = o3d.geometry.Image(np.ascontiguousarray(depth_np))
#         except Exception as e:
#             self.get_logger().error(f"Open3D image creation failed: {e}")
#             return

#         try:
#             rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
#                 color_o3d, depth_o3d,
#                 depth_scale=depth_scale,
#                 depth_trunc=self.depth_trunc,
#                 convert_rgb_to_intensity=False
#             )
#         except Exception as e:
#             self.get_logger().error(f"RGBD creation failed: {e}")
#             return

#         # --- Get odometry pose ---
#         T_w_body = self.odom_to_matrix(odom_msg)
#         T_w_cam = T_w_body @ self.T_body_cam

#         # --- Fuse ---
#         try:
#             pcd_cam = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)
#             flip_rosopt_to_open3d = np.array([
#                 [1, 0, 0, 0],
#                 [0,-1, 0, 0],
#                 [0, 0,-1, 0],
#                 [0, 0, 0, 1]
#             ])
#             pcd_cam.transform(flip_rosopt_to_open3d)
#             pcd_cam.transform(T_w_cam)
#             #self.global_pcd += pcd_cam
#         except Exception as e:
#             self.get_logger().warn(f"Open3D processing failed: {e}")

#         # extrinsic = np.linalg.inv(T_w_cam)
#         # self.volume.integrate(rgbd, self.intrinsics, extrinsic)

#         # --- Save pose record ---
#         row = [self.frame_id, f"{rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec:09d}"]
#         row.extend(T_w_cam.reshape(-1).tolist())
#         self.pose_records.append(row)

#         # === NEW: add downsampled copy of pcd_cam to buffer for ICP ===
#         try:
#             # create a copy / downsample to keep memory bounded
#             tmp = pcd_cam.voxel_down_sample(voxel_size=self.icp_downsample_voxel)
#             # open3d pointcloud has clone in recent versions; fallback to deepcopy
#             try:
#                 pcd_copy = tmp.clone()
#             except Exception:
#                 pcd_copy = deepcopy(tmp)
#             with self.buffer_lock:
#                 self.pcd_buffer.append(pcd_copy)
#                 # keep buffer bounded: drop oldest if needed
#                 if len(self.pcd_buffer) > self.pcd_buffer_size:
#                     # drop oldest
#                     self.pcd_buffer.pop(0)
#             # signal ICP thread
#             self.new_frame_event.set()
#         except Exception as e:
#             self.get_logger().warn(f"Failed to push pcd to ICP buffer: {e}")

#         self.frame_id += 1
    
#     # === NEW: ICP worker that registers the buffer into a merged cloud, then aligns to global_pcd ===
#     def _local_merge_worker(self):
#         """
#         Producer: waits for new frames in self.pcd_buffer, then (hybrid: N frames or T seconds)
#         merges them using Tukey loss (strict) to produce a local submap and pushes it into
#         self.merged_buffer for global alignment.
#         """
#         self.get_logger().info("Local ICP (merge) thread started.")
#         def make_local_loss(scale):
#             return o3d.pipelines.registration.HuberLoss(scale)

#         while not self.shutdown_event.is_set():
#             # Wait until a new frame arrives or timeout
#             self.new_frame_event.wait(self.icp_run_interval)
#             self.new_frame_event.clear()
#             if self.shutdown_event.is_set():
#                 break

#             # snapshot buffer
#             with self.buffer_lock:
#                 buffer_snapshot = [pcd.clone() if hasattr(pcd, "clone") else deepcopy(pcd)
#                                    for pcd in self.pcd_buffer]

#             if len(buffer_snapshot) == 0:
#                 continue

#             # Decide whether to flush a submap based on hybrid rule:
#             # - if we have >= N frames, or
#             # - if T seconds elapsed since last submap push
#             now = time.time()
#             if len(buffer_snapshot) < self.icp_submap_size and (now - self._last_submap_time) < self.icp_submap_time:
#                 # Not enough frames yet and not timed out -> continue waiting
#                 continue

#             try:
#                 # --- Merge buffer_snapshot into a local "merged" submap using Tukey ---
#                 if len(buffer_snapshot) == 1:
#                     merged = buffer_snapshot[0]
#                 else:
#                     target = buffer_snapshot[0]
#                     target_down = target.voxel_down_sample(self.icp_voxel_size)
#                     target_down.estimate_normals(
#                         o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30)
#                     )

#                     merged = target_down
#                     for i in range(1, len(buffer_snapshot)):
#                         source = buffer_snapshot[i]
#                         source_down = source.voxel_down_sample(self.icp_voxel_size)
#                         source_down.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30)
#                         )

#                         reg = o3d.pipelines.registration.registration_icp(
#                             source_down, merged,
#                             self.icp_max_corr,
#                             np.eye(4),
#                             o3d.pipelines.registration.TransformationEstimationPointToPlane(
#                                 make_local_loss(self.icp_voxel_size))
#                         )
#                         T = reg.transformation
#                         source_transformed = deepcopy(source).transform(T)
#                         merged += source_transformed
#                         merged = merged.voxel_down_sample(self.icp_downsample_voxel)
#                         merged.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_downsample_voxel*2, max_nn=30)
#                         )

#                 # push merged into merged_buffer (consumer will process)
#                 try:
#                     merged_copy = merged.clone() if hasattr(merged, "clone") else deepcopy(merged)
#                 except Exception:
#                     merged_copy = deepcopy(merged)

#                 with self.merged_lock:
#                     self.merged_buffer.append(merged_copy)
#                     # signal the global worker
#                     self.new_merged_event.set()

#                 # drop the frames we just merged from the pcd_buffer to avoid reprocessing them
#                 with self.buffer_lock:
#                     # remove up to len(buffer_snapshot) oldest entries (if they still exist)
#                     n_drop = min(len(buffer_snapshot), len(self.pcd_buffer))
#                     if n_drop > 0:
#                         self.pcd_buffer = self.pcd_buffer[n_drop:]

#                 self._last_submap_time = time.time()
#                 self.get_logger().info(f"Local merge: produced submap from {len(buffer_snapshot)} frames -> merged_buffer size {len(self.merged_buffer)}.")
#             except Exception as e:
#                 self.get_logger().warn(f"Local merge worker failed: {e}")
#                 time.sleep(0.05)

#         self.get_logger().info("Local merge thread exiting.")


#     def _global_align_worker(self):
#         """
#         Consumer: waits for merged submaps in self.merged_buffer, aligns each submap to
#         the global_pcd using multi-scale ICP with Huber loss, then updates global_pcd.
#         """
#         self.get_logger().info("Global ICP (align) thread started.")
#         last_global_transform = np.eye(4)

#         def make_global_loss(scale):
#             return o3d.pipelines.registration.TukeyLoss(scale)

#         while not self.shutdown_event.is_set():
#             # Wait for a new merged submap or timeout (so we can exit cleanly)
#             self.new_merged_event.wait(self.icp_run_interval)
#             self.new_merged_event.clear()
#             if self.shutdown_event.is_set():
#                 break

#             # pop one merged submap if available
#             with self.merged_lock:
#                 if len(self.merged_buffer) == 0:
#                     continue
#                 merged = self.merged_buffer.pop(0)

#             try:
#                 # prepare global copy
#                 with self.buffer_lock:
#                     global_copy = None
#                     if len(self.global_pcd.points) > 0:
#                         global_copy = self.global_pcd.voxel_down_sample(self.icp_voxel_size)
#                         global_copy.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30)
#                         )

#                 if global_copy is not None:
#                     # multi-scale ICP (coarse → fine) using Huber loss
#                     voxel_radii = [self.icp_voxel_size*4, self.icp_voxel_size*2, self.icp_voxel_size]
#                     max_iters = [50, 30, 14]
#                     current_trans = last_global_transform
#                     for scale, max_iter in zip(voxel_radii, max_iters):
#                         src_down = merged.voxel_down_sample(scale)
#                         tgt_down = global_copy.voxel_down_sample(scale)
#                         src_down.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=scale*2, max_nn=30))
#                         tgt_down.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=scale*2, max_nn=30))

#                         reg = o3d.pipelines.registration.registration_icp(
#                             src_down, tgt_down,
#                             self.icp_max_corr * 6.5,
#                             current_trans,
#                             o3d.pipelines.registration.TransformationEstimationPointToPlane(
#                                 make_global_loss(scale)),
#                             o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
#                         )
#                         current_trans = reg.transformation

#                     last_global_transform = current_trans
#                     merged.transform(current_trans)

#                 # integrate merged into global_pcd
#                 with self.buffer_lock:
#                     if len(self.global_pcd.points) == 0:
#                         self.global_pcd = merged
#                     else:
#                         self.global_pcd += merged
#                         try:
#                             self.global_pcd = self.global_pcd.voxel_down_sample(self.icp_downsample_voxel)
#                         except Exception:
#                             pass

#                 self.get_logger().info(f"Global align: integrated submap (merged_buffer size {len(self.merged_buffer)}) -> global points {len(self.global_pcd.points)}.")
#             except Exception as e:
#                 self.get_logger().warn(f"Global align worker failed: {e}")
#                 time.sleep(0.1)

#         self.get_logger().info("Global align thread exiting.")


#     def save_results(self):
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         out_dir = f"tsdf_output_{timestamp}"
#         os.makedirs(out_dir, exist_ok=True)

#         # Save final global_pcd
#         o3d.io.write_point_cloud(os.path.join(out_dir, "global_pcd.ply"), self.global_pcd)

#         # Save poses
#         with open(os.path.join(out_dir, "poses.csv"), "w") as f:
#             writer = csv.writer(f)
#             writer.writerow(["frame_id", "timestamp"] + [f"T{i}{j}" for i in range(4) for j in range(4)])
#             writer.writerows(self.pose_records)

#         if rclpy.ok():
#             self.get_logger().info(f"Saved results to {out_dir}")



# def main(args=None):
#     rclpy.init(args=args)
#     node = TSDFFusionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # signal ICP worker to exit and wait for it
#         try:
#             node.shutdown_event.set()
#             node.new_frame_event.set()  # wake the thread if it's waiting
#             if node.icp_thread.is_alive():
#                 node.get_logger().info("Waiting for ICP thread to finish...")
#                 node.icp_thread.join(timeout=10.0)
#         except Exception:
#             pass

#     # Run final color ICP and save results **before ROS shutdown**
#     node.save_results()

#     # Now destroy node and shutdown ROS
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
import cv2
import csv
import os
from datetime import datetime

# === NEW imports ===
import threading
import time
from copy import deepcopy

class TSDFFusionNode(Node):
    def __init__(self):
        super().__init__('tsdf_fusion_with_odometry')

        # ---- Parameters ----
        self.declare_parameter("rgb_topic", "/ascamera_hp60c/camera_publisher/rgb0/image")
        self.declare_parameter("depth_topic", "/ascamera_hp60c/camera_publisher/depth0/image_raw")
        self.declare_parameter("odom_topic", "/odometry_rect")
        self.declare_parameter("camera_frame", "cam0")
        # new: buffer size and icp settings
        self.declare_parameter("pcd_buffer_size", 14)  # number of recent pcds to keep for ICP
        self.declare_parameter("icp_voxel_size", 0.018)
        self.declare_parameter("icp_max_correspondence_distance", 0.35) #0.35
        self.declare_parameter("icp_downsample_voxel", 0.018)
        self.declare_parameter("icp_run_interval", 1.0)  # seconds between ICP runs (if no new frames waits up to this)

        # --- New ICP tuning + acceptance params ---
        self.declare_parameter("icp_submap_size", 14)
        self.declare_parameter("icp_submap_time", 4.0)
        self.declare_parameter("icp_accept_fitness", 0.3)
        self.declare_parameter("icp_max_rmse", 0.12)
        self.declare_parameter("icp_max_attempts", 2)
        self.declare_parameter("icp_overlap_threshold", 0.27) #0.42
        self.declare_parameter("icp_adaptive_submap_factor", 0.5)
        self.declare_parameter("icp_max_corr_scale", 2.8)  # coarse-scale multiplier
        self.declare_parameter("use_submap_graph_mode", False)

        rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

        # read icp params
        self.pcd_buffer_size = self.get_parameter("pcd_buffer_size").get_parameter_value().integer_value
        self.icp_voxel_size = self.get_parameter("icp_voxel_size").get_parameter_value().double_value
        self.icp_max_corr = self.get_parameter("icp_max_correspondence_distance").get_parameter_value().double_value
        self.icp_downsample_voxel = self.get_parameter("icp_downsample_voxel").get_parameter_value().double_value
        self.icp_run_interval = self.get_parameter("icp_run_interval").get_parameter_value().double_value

        # read new params
        self.icp_submap_size = self.get_parameter("icp_submap_size").get_parameter_value().integer_value
        self.icp_submap_time = self.get_parameter("icp_submap_time").get_parameter_value().double_value
        self.icp_accept_fitness = self.get_parameter("icp_accept_fitness").get_parameter_value().double_value
        self.icp_max_rmse = self.get_parameter("icp_max_rmse").get_parameter_value().double_value
        self.icp_max_attempts = self.get_parameter("icp_max_attempts").get_parameter_value().integer_value
        self.icp_overlap_threshold = self.get_parameter("icp_overlap_threshold").get_parameter_value().double_value
        self.icp_adaptive_submap_factor = self.get_parameter("icp_adaptive_submap_factor").get_parameter_value().double_value
        self.icp_max_corr_scale = self.get_parameter("icp_max_corr_scale").get_parameter_value().double_value
        self.use_submap_graph_mode = self.get_parameter("use_submap_graph_mode").get_parameter_value().bool_value

        # ---- Static extrinsic: body(imu) → cam0 ----
        self.T_body_cam = np.array([
            [-0.02156132, -0.99483675, -0.09917134,  0.03432751],
            [-0.25782609,  0.10137213, -0.96085868,  0.03072772],
            [ 0.96595073,  0.00485158, -0.25868058, -0.07285357],
            [ 0.,          0.,          0.,          1.        ]
        ])

        # ---- Intrinsics (replace with your calibrated values) ----
        fx, fy, cx, cy = 525.0, 525.0, 319.5, 239.5
        width, height = 640, 480
        self.intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

        # ---- TSDF volume (disabled for now) ----

        # ---- Subscribers ----
        self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.odom_sub],
            queue_size=10, slop=0.01
        )
        self.ts.registerCallback(self.sync_callback)

        # ---- Helpers ----
        self.bridge = CvBridge()
        self.frame_id = 0
        self.pose_records = []
        self.global_pcd = o3d.geometry.PointCloud()
        self.depth_trunc = 2.0
        self.min_valid_depth_pixels = 500  # skip frames with almost no depth

        # track last odometry/camera pose (optional prior)
        self.last_odom_transform = None

        # submap graph for later refinement
        self.submap_graph = []

        # === ICP buffer & threading ===
        self.pcd_buffer = []  # list of recent pointclouds (o3d.geometry.PointCloud)
        self.buffer_lock = threading.Lock()
        self.new_frame_event = threading.Event()
        self.shutdown_event = threading.Event()

        # --- NEW: merged submap buffer for two-stage pipeline ---
        self.merged_buffer = []           # list of tuples (merged_submap, attempts)
        self.merged_lock = threading.Lock()
        self.new_merged_event = threading.Event()

        # track last submap push time to support the hybrid (N or T) rule
        self._last_submap_time = time.time()

        # start ICP threads: local (producer) + global (consumer)
        self.local_thread = threading.Thread(target=self._local_merge_worker, daemon=True)
        self.global_thread = threading.Thread(target=self._global_align_worker, daemon=True)
        # keep icp_thread attribute for backwards compatibility with external shutdown code
        self.icp_thread = self.local_thread
        self.local_thread.start()
        self.global_thread.start()

        self.get_logger().info("TSDF fusion node initialized (RGBD + odometry_rect). ICP worker started.")

    def _depth_msg_to_numpy(self, depth_msg, depth_cv_raw):
        """Convert depth cv2 image to numpy with correct dtype + scale."""
        encoding = depth_msg.encoding.lower()
        if encoding in ("16uc1", "mono16"):
            depth_np = np.asarray(depth_cv_raw, dtype=np.uint16)
            depth_scale = 1000.0
        elif encoding in ("32fc1", "float32"):
            depth_np = np.asarray(depth_cv_raw, dtype=np.float32)
            depth_scale = 1.0
        else:
            # fallback: try uint16
            depth_np = np.asarray(depth_cv_raw).astype(np.uint16)
            depth_scale = 1000.0
        return depth_np, depth_scale

    def odom_to_matrix(self, odom: Odometry):
        """Convert nav_msgs/Odometry pose to 4x4 transformation matrix."""
        pose = odom.pose.pose
        tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
        qx, qy, qz, qw = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

        norm = qw*qw + qx*qx + qy*qy + qz*qz
        s = 1.0 / np.sqrt(norm)
        qw, qx, qy, qz = qw*s, qx*s, qy*s, qz*s
        R = np.array([
            [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
            [2*(qx*qy+qz*qw),     1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
            [2*(qx*qz-qy*qw),     2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]
        ], dtype=np.float64)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.array([tx, ty, tz])
        return T

    def sync_callback(self, rgb_msg: Image, depth_msg: Image, odom_msg: Odometry):
        # --- Convert RGB ---
        try:
            rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB cv_bridge failed: {e}")
            return

        # --- Convert Depth (robust) ---
        try:
            depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth cv_bridge passthrough failed: {e}")
            try:
                depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
            except Exception as e2:
                self.get_logger().error(f"Depth cv_bridge fallback failed: {e2}")
                return

        try:
            depth_np, depth_scale = self._depth_msg_to_numpy(depth_msg, depth_cv_raw)
        except Exception as e:
            self.get_logger().error(f"Depth -> numpy sanitization failed: {e}")
            return

        valid = int(np.count_nonzero(depth_np))
        if valid < self.min_valid_depth_pixels:
            self.get_logger().warn(f"Skipping frame {self.frame_id}: too few valid depth pixels ({valid}).")
            return
        
        # resize RGB if mismatch
        if rgb_cv.shape[0] != self.intrinsics.height or rgb_cv.shape[1] != self.intrinsics.width:
            rgb_cv = cv2.resize(rgb_cv, (self.intrinsics.width, self.intrinsics.height))

        # --- Build Open3D RGB + Depth ---
        try:
            color_o3d = o3d.geometry.Image(cv2.cvtColor(np.ascontiguousarray(rgb_cv), cv2.COLOR_BGR2RGB))
            depth_o3d = o3d.geometry.Image(np.ascontiguousarray(depth_np))
        except Exception as e:
            self.get_logger().error(f"Open3D image creation failed: {e}")
            return

        try:
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_o3d, depth_o3d,
                depth_scale=depth_scale,
                depth_trunc=self.depth_trunc,
                convert_rgb_to_intensity=False
            )
        except Exception as e:
            self.get_logger().error(f"RGBD creation failed: {e}")
            return

        # --- Get odometry pose ---
        T_w_body = self.odom_to_matrix(odom_msg)
        T_w_cam = T_w_body @ self.T_body_cam

        # store last odom/camera transform (useful as a prior)
        self.last_odom_transform = T_w_cam

        # --- Fuse ---
        try:
            pcd_cam = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)
            flip_rosopt_to_open3d = np.array([
                [1, 0, 0, 0],
                [0,-1, 0, 0],
                [0, 0,-1, 0],
                [0, 0, 0, 1]
            ])
            pcd_cam.transform(flip_rosopt_to_open3d)
            pcd_cam.transform(T_w_cam)
            #self.global_pcd += pcd_cam
        except Exception as e:
            self.get_logger().warn(f"Open3D processing failed: {e}")

        # --- Save pose record ---
        row = [self.frame_id, f"{rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec:09d}"]
        row.extend(T_w_cam.reshape(-1).tolist())
        self.pose_records.append(row)

        # === NEW: add downsampled copy of pcd_cam to buffer for ICP ===
        try:
            # create a copy / downsample to keep memory bounded
            tmp = pcd_cam.voxel_down_sample(voxel_size=self.icp_downsample_voxel)
            # open3d pointcloud has clone in recent versions; fallback to deepcopy
            try:
                pcd_copy = tmp.clone()
            except Exception:
                pcd_copy = deepcopy(tmp)
            with self.buffer_lock:
                self.pcd_buffer.append(pcd_copy)
                # keep buffer bounded: drop oldest if needed
                if len(self.pcd_buffer) > self.pcd_buffer_size:
                    # drop oldest
                    self.pcd_buffer.pop(0)
            # signal ICP thread
            self.new_frame_event.set()
        except Exception as e:
            self.get_logger().warn(f"Failed to push pcd to ICP buffer: {e}")

        self.frame_id += 1
    
    # === NEW: ICP worker that registers the buffer into a merged cloud, then aligns to global_pcd ===
    def _local_merge_worker(self):
        """
        Producer: waits for new frames in self.pcd_buffer, then (hybrid: N frames or T seconds)
        merges them using Tukey loss (strict) to produce a local submap and pushes it into
        self.merged_buffer for global alignment.
        """
        self.get_logger().info("Local ICP (merge) thread started.")

        def make_local_loss(scale):
            return o3d.pipelines.registration.HuberLoss(scale)

        def estimate_overlap(first, last, radius):
            """Rough fraction of points in `first` that have a neighbor in `last` within `radius`.
            We use small voxel-downsampled clouds for speed."""
            try:
                a = first.voxel_down_sample(self.icp_voxel_size)
                b = last.voxel_down_sample(self.icp_voxel_size)
                if len(a.points) == 0 or len(b.points) == 0:
                    return 0.0
                kdt = o3d.geometry.KDTreeFlann(b)
                pts_a = np.asarray(a.points)
                close = 0
                # sample if very dense to keep runtime bounded
                max_checks = 512
                step = max(1, pts_a.shape[0] // max_checks)
                for p in pts_a[::step]:
                    _k, idx, _ = kdt.search_knn_vector_3d(p, 1)
                    if _k > 0:
                        bpts = np.asarray(b.points)
                        if np.linalg.norm(p - bpts[idx[0]]) <= radius:
                            close += 1
                total = max(1, int(np.ceil(float(pts_a.shape[0]) / step)))
                return float(close) / float(total)
            except Exception:
                return 0.0

        while not self.shutdown_event.is_set():
            # Wait until a new frame arrives or timeout
            self.new_frame_event.wait(self.icp_run_interval)
            self.new_frame_event.clear()
            if self.shutdown_event.is_set():
                break

            # snapshot buffer
            with self.buffer_lock:
                buffer_snapshot = [pcd.clone() if hasattr(pcd, "clone") else deepcopy(pcd)
                                   for pcd in self.pcd_buffer]

            if len(buffer_snapshot) == 0:
                continue

            # adaptive submap sizing: if first/last overlap is low, create smaller submaps
            try:
                overlap_radius = max(1e-3, self.icp_max_corr * 1.5)
                overlap_est = estimate_overlap(buffer_snapshot[0], buffer_snapshot[-1], overlap_radius)
            except Exception:
                overlap_est = 1.0

            if overlap_est < self.icp_overlap_threshold:
                effective_submap_size = max(1, int(self.icp_submap_size * self.icp_adaptive_submap_factor))
            else:
                effective_submap_size = self.icp_submap_size

            # Decide whether to flush a submap based on hybrid rule (N frames or T seconds)
            now = time.time()
            if len(buffer_snapshot) < effective_submap_size and float(now - self._last_submap_time) < float(self.icp_submap_time):
                # Not enough frames yet and not timed out -> continue waiting
                continue

            try:
                # --- Merge buffer_snapshot into a local "merged" submap using Tukey ---
                if len(buffer_snapshot) == 1:
                    merged = buffer_snapshot[0]
                else:
                    target = buffer_snapshot[0]
                    target_down = target.voxel_down_sample(self.icp_voxel_size)
                    target_down.estimate_normals(
                        o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30)
                    )

                    merged = target_down
                    for i in range(1, len(buffer_snapshot)):
                        source = buffer_snapshot[i]
                        source_down = source.voxel_down_sample(self.icp_voxel_size)
                        source_down.estimate_normals(
                            o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30)
                        )

                        reg = o3d.pipelines.registration.registration_icp(
                            source_down, merged,
                            self.icp_max_corr,
                            np.eye(4),
                            o3d.pipelines.registration.TransformationEstimationPointToPlane(
                                make_local_loss(self.icp_voxel_size))
                        )
                        T = reg.transformation
                        source_transformed = deepcopy(source).transform(T)
                        merged += source_transformed
                        merged = merged.voxel_down_sample(self.icp_downsample_voxel)
                        merged.estimate_normals(
                            o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_downsample_voxel*2, max_nn=30)
                        )

                # push merged into merged_buffer (store as tuple (pcd, attempts))
                try:
                    merged_copy = merged.clone() if hasattr(merged, "clone") else deepcopy(merged)
                except Exception:
                    merged_copy = deepcopy(merged)

                with self.merged_lock:
                    self.merged_buffer.append((merged_copy, 0))
                    # signal the global worker
                    self.new_merged_event.set()

                # drop the frames we just merged from the pcd_buffer to avoid reprocessing them
                with self.buffer_lock:
                    # remove up to len(buffer_snapshot) oldest entries (if they still exist)
                    n_drop = min(len(buffer_snapshot), len(self.pcd_buffer))
                    if n_drop > 0:
                        self.pcd_buffer = self.pcd_buffer[n_drop:]

                self._last_submap_time = time.time()
                self.get_logger().info(f"Local merge: produced submap from {len(buffer_snapshot)} frames -> merged_buffer size {len(self.merged_buffer)}.")
            except Exception as e:
                self.get_logger().warn(f"Local merge worker failed: {e}")
                time.sleep(0.05)

        self.get_logger().info("Local merge thread exiting.")


    def _global_align_worker(self):
        """
        Consumer: waits for merged submaps in self.merged_buffer, aligns each submap to
        the global_pcd using multi-scale ICP with Huber loss, then updates global_pcd.

        This variant:
          - uses a single start (last_global_transform)
          - adaptive max correspondence per scale
          - accept/reject with requeue attempts
          - stores accepted submaps into self.submap_graph for later refinement
        """
        self.get_logger().info("Global ICP (align) thread started.")
        last_global_transform = np.eye(4)

        def make_global_loss(scale):
            return o3d.pipelines.registration.TukeyLoss(scale)

        while not self.shutdown_event.is_set():
            # Wait for a new merged submap or timeout (so we can exit cleanly)
            self.new_merged_event.wait(self.icp_run_interval)
            self.new_merged_event.clear()
            if self.shutdown_event.is_set():
                break

            # pop one merged submap if available
            with self.merged_lock:
                if len(self.merged_buffer) == 0:
                    continue
                item = self.merged_buffer.pop(0)

            # item may be (pcd, attempts)
            if isinstance(item, tuple) and len(item) == 2:
                merged, attempts = item
            else:
                merged = item
                attempts = 0

            try:
                # keep an original copy for requeue/integration
                orig_merged = merged.clone() if hasattr(merged, "clone") else deepcopy(merged)

                # prepare a (downsampled) copy of the global map for fast ICP
                with self.buffer_lock:
                    global_copy = None
                    if len(self.global_pcd.points) > 0:
                        global_copy = self.global_pcd.voxel_down_sample(self.icp_voxel_size)
                        global_copy.estimate_normals(
                            o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30)
                        )

                if global_copy is not None:
                    # multi-scale ICP (coarse -> fine) using Huber loss
                    voxel_radii = [self.icp_voxel_size*4, self.icp_voxel_size*2, self.icp_voxel_size]
                    max_iters = [50, 30, 14]

                    current_trans = last_global_transform
                    reg = None
                    for scale, max_iter in zip(voxel_radii, max_iters):
                        src_down = merged.voxel_down_sample(scale)
                        tgt_down = global_copy.voxel_down_sample(scale)
                        src_down.estimate_normals(
                            o3d.geometry.KDTreeSearchParamHybrid(radius=scale*2, max_nn=30))
                        tgt_down.estimate_normals(
                            o3d.geometry.KDTreeSearchParamHybrid(radius=scale*2, max_nn=30))

                        # adaptive max correspondence distance (scale-sensitive)
                        base_corr = max(1e-6, float(self.icp_max_corr))
                        scale_multiplier = max(1.0, scale / max(1e-6, float(self.icp_voxel_size)))
                        corr = base_corr * scale_multiplier * float(self.icp_max_corr_scale)

                        reg = o3d.pipelines.registration.registration_icp(
                            src_down, tgt_down,
                            corr,
                            current_trans,
                            o3d.pipelines.registration.TransformationEstimationPointToPlane(
                                make_global_loss(scale)),
                            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
                        )
                        current_trans = reg.transformation

                    # final registration result is in `reg` (from last scale)
                    best_fitness = reg.fitness if reg is not None else 0.0
                    best_rmse = reg.inlier_rmse if reg is not None else float('inf')

                    # acceptance test
                    if reg is None or best_fitness < float(self.icp_accept_fitness) or best_rmse > float(self.icp_max_rmse):
                        attempts += 1
                        if attempts <= int(self.icp_max_attempts):
                            with self.merged_lock:
                                self.merged_buffer.append((orig_merged, attempts))
                            self.get_logger().warn(f"Global align: rejected submap (fitness={best_fitness:.3f}, rmse={best_rmse:.4f}), requeued (attempt {attempts}).")
                            continue
                        else:
                            self.get_logger().warn(f"Global align: dropped submap after {attempts} attempts (fitness={best_fitness:.3f}, rmse={best_rmse:.4f}).")
                            continue

                    # accepted: apply transform
                    orig_merged.transform(current_trans)
                    last_global_transform = current_trans
                else:
                    # empty global -> insert first submap as-is
                    self.get_logger().info("Global align: empty global map, inserting first submap.")
                    # no ICP, keep orig_merged coordinates
                    last_global_transform = last_global_transform

                # bookkeeping: keep submaps separately (for later refinement)
                try:
                    submap_entry = {'pcd': orig_merged.clone() if hasattr(orig_merged, 'clone') else deepcopy(orig_merged),
                                    'pose': last_global_transform}
                except Exception:
                    submap_entry = {'pcd': deepcopy(orig_merged), 'pose': last_global_transform}

                self.submap_graph.append(submap_entry)

                with self.buffer_lock:
                    if self.use_submap_graph_mode:
                        # graph-only: defer fusion
                        pass
                    else:
                        if len(self.global_pcd.points) == 0:
                            self.global_pcd = orig_merged
                        else:
                            self.global_pcd += orig_merged
                            try:
                                self.global_pcd = self.global_pcd.voxel_down_sample(self.icp_downsample_voxel)
                            except Exception:
                                pass

                self.get_logger().info(f"Global align: integrated submap (merged_buffer size {len(self.merged_buffer)}) -> global points {len(self.global_pcd.points)}.")
            except Exception as e:
                self.get_logger().warn(f"Global align worker failed: {e}")
                time.sleep(0.1)

        self.get_logger().info("Global align thread exiting.")


    def save_results(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = f"tsdf_output_{timestamp}"
        os.makedirs(out_dir, exist_ok=True)

        # Save final global_pcd
        o3d.io.write_point_cloud(os.path.join(out_dir, "global_pcd.ply"), self.global_pcd)

        # Save poses
        with open(os.path.join(out_dir, "poses.csv"), "w") as f:
            writer = csv.writer(f)
            writer.writerow(["frame_id", "timestamp"] + [f"T{i}{j}" for i in range(4) for j in range(4)])
            writer.writerows(self.pose_records)

        if rclpy.ok():
            self.get_logger().info(f"Saved results to {out_dir}")



def main(args=None):
    rclpy.init(args=args)
    node = TSDFFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # signal ICP worker to exit and wait for it
        try:
            node.shutdown_event.set()
            node.new_frame_event.set()  # wake the thread if it's waiting
            node.new_merged_event.set()
            if node.icp_thread.is_alive():
                node.get_logger().info("Waiting for ICP thread to finish...")
                node.icp_thread.join(timeout=10.0)
            if node.global_thread.is_alive():
                node.get_logger().info("Waiting for global ICP thread to finish...")
                node.global_thread.join(timeout=10.0)
        except Exception:
            pass

    # Run final color ICP and save results **before ROS shutdown**
    node.save_results()

    # Now destroy node and shutdown ROS
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# import rclpy
# from rclpy.node import Node
# import message_filters
# from sensor_msgs.msg import Image
# from nav_msgs.msg import Odometry
# from cv_bridge import CvBridge
# import numpy as np
# import open3d as o3d
# import cv2
# import csv
# import os
# from datetime import datetime

# # === NEW imports ===
# import threading
# import time
# from copy import deepcopy

# class TSDFFusionNode(Node):
#     def __init__(self):
#         super().__init__('tsdf_fusion_with_odometry')

#         # ---- Parameters ----
#         self.declare_parameter("rgb_topic", "/ascamera_hp60c/camera_publisher/rgb0/image")
#         self.declare_parameter("depth_topic", "/ascamera_hp60c/camera_publisher/depth0/image_raw")
#         self.declare_parameter("odom_topic", "/odometry_rect")
#         self.declare_parameter("camera_frame", "cam0")
#         # new: buffer size and icp settings
#         self.declare_parameter("pcd_buffer_size", 13)  # number of recent pcds to keep for ICP
#         self.declare_parameter("icp_voxel_size", 0.02)
#         self.declare_parameter("icp_max_correspondence_distance", 0.3)
#         self.declare_parameter("icp_downsample_voxel", 0.02)
#         self.declare_parameter("icp_run_interval", 1.0)  # seconds between ICP runs (if no new frames waits up to this)

#         # --- New ICP tuning + acceptance params ---
#         self.declare_parameter("icp_submap_size", 13)
#         self.declare_parameter("icp_submap_time", 2.5)
#         self.declare_parameter("icp_accept_fitness", 0.3)
#         self.declare_parameter("icp_max_rmse", 0.2)
#         self.declare_parameter("icp_max_attempts", 2)
#         self.declare_parameter("icp_overlap_threshold", 0.3)
#         self.declare_parameter("icp_adaptive_submap_factor", 0.4)
#         self.declare_parameter("icp_max_corr_scale", 2.7)  # coarse-scale multiplier
#         self.declare_parameter("use_submap_graph_mode", False)

#         # --- Adaptive downsampling params (near/mid/far bins) ---
#         self.declare_parameter("adaptive_near_dist", 0.8)   # meters
#         self.declare_parameter("adaptive_mid_dist", 2.0)    # meters
#         self.declare_parameter("adaptive_near_voxel", 0.01)
#         self.declare_parameter("adaptive_mid_voxel", 0.02)
#         self.declare_parameter("adaptive_far_voxel", 0.04)
#         # Normalize densities after merging
#         self.declare_parameter("icp_normalize_voxel", 0.02)

#         rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
#         depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
#         odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
#         self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

#         # read icp params
#         self.pcd_buffer_size = self.get_parameter("pcd_buffer_size").get_parameter_value().integer_value
#         self.icp_voxel_size = self.get_parameter("icp_voxel_size").get_parameter_value().double_value
#         self.icp_max_corr = self.get_parameter("icp_max_correspondence_distance").get_parameter_value().double_value
#         self.icp_downsample_voxel = self.get_parameter("icp_downsample_voxel").get_parameter_value().double_value
#         self.icp_run_interval = self.get_parameter("icp_run_interval").get_parameter_value().double_value

#         # read new params
#         self.icp_submap_size = self.get_parameter("icp_submap_size").get_parameter_value().integer_value
#         self.icp_submap_time = self.get_parameter("icp_submap_time").get_parameter_value().double_value
#         self.icp_accept_fitness = self.get_parameter("icp_accept_fitness").get_parameter_value().double_value
#         self.icp_max_rmse = self.get_parameter("icp_max_rmse").get_parameter_value().double_value
#         self.icp_max_attempts = self.get_parameter("icp_max_attempts").get_parameter_value().integer_value
#         self.icp_overlap_threshold = self.get_parameter("icp_overlap_threshold").get_parameter_value().double_value
#         self.icp_adaptive_submap_factor = self.get_parameter("icp_adaptive_submap_factor").get_parameter_value().double_value
#         self.icp_max_corr_scale = self.get_parameter("icp_max_corr_scale").get_parameter_value().double_value
#         self.use_submap_graph_mode = self.get_parameter("use_submap_graph_mode").get_parameter_value().bool_value

#         # read adaptive downsample params
#         self.adaptive_near_dist = self.get_parameter("adaptive_near_dist").get_parameter_value().double_value
#         self.adaptive_mid_dist = self.get_parameter("adaptive_mid_dist").get_parameter_value().double_value
#         self.adaptive_near_voxel = self.get_parameter("adaptive_near_voxel").get_parameter_value().double_value
#         self.adaptive_mid_voxel = self.get_parameter("adaptive_mid_voxel").get_parameter_value().double_value
#         self.adaptive_far_voxel = self.get_parameter("adaptive_far_voxel").get_parameter_value().double_value
#         self.icp_normalize_voxel = self.get_parameter("icp_normalize_voxel").get_parameter_value().double_value

#         # ---- Static extrinsic: body(imu) → cam0 ----
#         self.T_body_cam = np.array([
#             [-0.02156132, -0.99483675, -0.09917134,  0.03432751],
#             [-0.25782609,  0.10137213, -0.96085868,  0.03072772],
#             [ 0.96595073,  0.00485158, -0.25868058, -0.07285357],
#             [ 0.,          0.,          0.,          1.        ]
#         ])

#         # ---- Intrinsics (replace with your calibrated values) ----
#         fx, fy, cx, cy = 525.0, 525.0, 319.5, 239.5
#         width, height = 640, 480
#         self.intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

#         # ---- TSDF volume (disabled for now) ----

#         # ---- Subscribers ----
#         self.rgb_sub = message_filters.Subscriber(self, Image, rgb_topic)
#         self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
#         self.odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)

#         self.ts = message_filters.ApproximateTimeSynchronizer(
#             [self.rgb_sub, self.depth_sub, self.odom_sub],
#             queue_size=10, slop=0.01
#         )
#         self.ts.registerCallback(self.sync_callback)

#         # ---- Helpers ----
#         self.bridge = CvBridge()
#         self.frame_id = 0
#         self.pose_records = []
#         self.global_pcd = o3d.geometry.PointCloud()
#         self.depth_trunc = 2.0
#         self.min_valid_depth_pixels = 500  # skip frames with almost no depth

#         # track last odometry/camera pose (optional prior)
#         self.last_odom_transform = None

#         # submap graph for later refinement
#         self.submap_graph = []

#         # === ICP buffer & threading ===
#         self.pcd_buffer = []  # list of recent pointclouds (o3d.geometry.PointCloud)
#         self.buffer_lock = threading.Lock()
#         self.new_frame_event = threading.Event()
#         self.shutdown_event = threading.Event()

#         # --- NEW: merged submap buffer for two-stage pipeline ---
#         self.merged_buffer = []           # list of tuples (merged_submap, attempts)
#         self.merged_lock = threading.Lock()
#         self.new_merged_event = threading.Event()

#         # track last submap push time to support the hybrid (N or T) rule
#         self._last_submap_time = time.time()

#         # start ICP threads: local (producer) + global (consumer)
#         self.local_thread = threading.Thread(target=self._local_merge_worker, daemon=True)
#         self.global_thread = threading.Thread(target=self._global_align_worker, daemon=True)
#         # keep icp_thread attribute for backwards compatibility with external shutdown code
#         self.icp_thread = self.local_thread
#         self.local_thread.start()
#         self.global_thread.start()

#         self.get_logger().info("TSDF fusion node initialized (RGBD + odometry_rect). ICP worker started.")

#     def _depth_msg_to_numpy(self, depth_msg, depth_cv_raw):
#         """Convert depth cv2 image to numpy with correct dtype + scale."""
#         encoding = depth_msg.encoding.lower()
#         if encoding in ("16uc1", "mono16"):
#             depth_np = np.asarray(depth_cv_raw, dtype=np.uint16)
#             depth_scale = 1000.0
#         elif encoding in ("32fc1", "float32"):
#             depth_np = np.asarray(depth_cv_raw, dtype=np.float32)
#             depth_scale = 1.0
#         else:
#             # fallback: try uint16
#             depth_np = np.asarray(depth_cv_raw).astype(np.uint16)
#             depth_scale = 1000.0
#         return depth_np, depth_scale

#     def odom_to_matrix(self, odom: Odometry):
#         """Convert nav_msgs/Odometry pose to 4x4 transformation matrix."""
#         pose = odom.pose.pose
#         tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
#         qx, qy, qz, qw = (
#             pose.orientation.x,
#             pose.orientation.y,
#             pose.orientation.z,
#             pose.orientation.w,
#         )

#         norm = qw*qw + qx*qx + qy*qy + qz*qz
#         s = 1.0 / np.sqrt(norm)
#         qw, qx, qy, qz = qw*s, qx*s, qy*s, qz*s
#         R = np.array([
#             [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
#             [2*(qx*qy+qz*qw),     1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
#             [2*(qx*qz-qy*qw),     2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]
#         ], dtype=np.float64)
#         T = np.eye(4)
#         T[:3, :3] = R
#         T[:3, 3] = np.array([tx, ty, tz])
#         return T

#     def _adaptive_downsample_camera(self, pcd_cam):
#         """Adaptive voxel downsampling based on distance (z) in camera frame.
#         Splits points into near / mid / far and downsamples each band with a
#         different voxel size, then merges them.
#         """
#         try:
#             pts = np.asarray(pcd_cam.points)
#             if pts.shape[0] == 0:
#                 return pcd_cam

#             has_colors = pcd_cam.has_colors()
#             has_normals = pcd_cam.has_normals()
#             colors = np.asarray(pcd_cam.colors) if has_colors else None
#             normals = np.asarray(pcd_cam.normals) if has_normals else None

#             z = pts[:, 2]

#             # define masks
#             near_mask = z < float(self.adaptive_near_dist)
#             mid_mask = (z >= float(self.adaptive_near_dist)) & (z < float(self.adaptive_mid_dist))
#             far_mask = z >= float(self.adaptive_mid_dist)

#             new_pcd = o3d.geometry.PointCloud()

#             bands = [ (near_mask, float(self.adaptive_near_voxel)),
#                       (mid_mask, float(self.adaptive_mid_voxel)),
#                       (far_mask, float(self.adaptive_far_voxel)) ]

#             for mask, voxel in bands:
#                 if np.count_nonzero(mask) == 0:
#                     continue
#                 sub = o3d.geometry.PointCloud()
#                 sub.points = o3d.utility.Vector3dVector(pts[mask])
#                 if has_colors:
#                     sub.colors = o3d.utility.Vector3dVector(colors[mask])
#                 if has_normals:
#                     sub.normals = o3d.utility.Vector3dVector(normals[mask])

#                 try:
#                     sub_down = sub.voxel_down_sample(voxel_size=voxel)
#                 except TypeError:
#                     # older open3d signature
#                     sub_down = sub.voxel_down_sample(voxel)

#                 # merge
#                 if len(new_pcd.points) == 0:
#                     new_pcd = sub_down
#                 else:
#                     new_pcd += sub_down

#             if len(new_pcd.points) == 0:
#                 return pcd_cam

#             return new_pcd
#         except Exception:
#             return pcd_cam

#     def _normalize_density(self, pcd):
#         """Uniform density resampling: simple voxel downsample with icp_normalize_voxel.
#         This regularizes dense close walls vs sparse far regions so ICP isn't biased.
#         """
#         try:
#             v = float(self.icp_normalize_voxel)
#             if v > 0:
#                 try:
#                     return pcd.voxel_down_sample(voxel_size=v)
#                 except TypeError:
#                     return pcd.voxel_down_sample(v)
#         except Exception:
#             pass
#         return pcd

#     def sync_callback(self, rgb_msg: Image, depth_msg: Image, odom_msg: Odometry):
#         # --- Convert RGB ---
#         try:
#             rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"RGB cv_bridge failed: {e}")
#             return

#         # --- Convert Depth (robust) ---
#         try:
#             depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Depth cv_bridge passthrough failed: {e}")
#             try:
#                 depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
#             except Exception as e2:
#                 self.get_logger().error(f"Depth cv_bridge fallback failed: {e2}")
#                 return

#         try:
#             depth_np, depth_scale = self._depth_msg_to_numpy(depth_msg, depth_cv_raw)
#         except Exception as e:
#             self.get_logger().error(f"Depth -> numpy sanitization failed: {e}")
#             return

#         valid = int(np.count_nonzero(depth_np))
#         if valid < self.min_valid_depth_pixels:
#             self.get_logger().warn(f"Skipping frame {self.frame_id}: too few valid depth pixels ({valid}).")
#             return
        
#         # resize RGB if mismatch
#         if rgb_cv.shape[0] != self.intrinsics.height or rgb_cv.shape[1] != self.intrinsics.width:
#             rgb_cv = cv2.resize(rgb_cv, (self.intrinsics.width, self.intrinsics.height))

#         # --- Build Open3D RGB + Depth ---
#         try:
#             color_o3d = o3d.geometry.Image(cv2.cvtColor(np.ascontiguousarray(rgb_cv), cv2.COLOR_BGR2RGB))
#             depth_o3d = o3d.geometry.Image(np.ascontiguousarray(depth_np))
#         except Exception as e:
#             self.get_logger().error(f"Open3D image creation failed: {e}")
#             return

#         try:
#             rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
#                 color_o3d, depth_o3d,
#                 depth_scale=depth_scale,
#                 depth_trunc=self.depth_trunc,
#                 convert_rgb_to_intensity=False
#             )
#         except Exception as e:
#             self.get_logger().error(f"RGBD creation failed: {e}")
#             return

#         # --- Get odometry pose ---
#         T_w_body = self.odom_to_matrix(odom_msg)
#         T_w_cam = T_w_body @ self.T_body_cam

#         # store last odom/camera transform (useful as a prior)
#         self.last_odom_transform = T_w_cam

#         # --- Fuse ---
#         try:
#             pcd_cam = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)

#             # --- Adaptive downsample in camera frame to preserve distant geometry ---
#             try:
#                 pcd_cam = self._adaptive_downsample_camera(pcd_cam)
#             except Exception as e:
#                 self.get_logger().warn(f"Adaptive downsample failed: {e}")

#             flip_rosopt_to_open3d = np.array([
#                 [1, 0, 0, 0],
#                 [0,-1, 0, 0],
#                 [0, 0,-1, 0],
#                 [0, 0, 0, 1]
#             ])
#             pcd_cam.transform(flip_rosopt_to_open3d)
#             pcd_cam.transform(T_w_cam)
#             #self.global_pcd += pcd_cam
#         except Exception as e:
#             self.get_logger().warn(f"Open3D processing failed: {e}")

#         # --- Save pose record ---
#         row = [self.frame_id, f"{rgb_msg.header.stamp.sec}.{rgb_msg.header.stamp.nanosec:09d}"]
#         row.extend(T_w_cam.reshape(-1).tolist())
#         self.pose_records.append(row)

#         # === NEW: add downsampled copy of pcd_cam to buffer for ICP ===
#         try:
#             # create a copy / downsample to keep memory bounded
#             try:
#                 tmp = pcd_cam.voxel_down_sample(voxel_size=self.icp_downsample_voxel)
#             except TypeError:
#                 tmp = pcd_cam.voxel_down_sample(self.icp_downsample_voxel)

#             # open3d pointcloud has clone in recent versions; fallback to deepcopy
#             try:
#                 pcd_copy = tmp.clone()
#             except Exception:
#                 pcd_copy = deepcopy(tmp)
#             with self.buffer_lock:
#                 self.pcd_buffer.append(pcd_copy)
#                 # keep buffer bounded: drop oldest if needed
#                 if len(self.pcd_buffer) > self.pcd_buffer_size:
#                     # drop oldest
#                     self.pcd_buffer.pop(0)
#             # signal ICP thread
#             self.new_frame_event.set()
#         except Exception as e:
#             self.get_logger().warn(f"Failed to push pcd to ICP buffer: {e}")

#         self.frame_id += 1
    
#     # === NEW: ICP worker that registers the buffer into a merged cloud, then aligns to global_pcd ===
#     def _local_merge_worker(self):
#         """
#         Producer: waits for new frames in self.pcd_buffer, then (hybrid: N frames or T seconds)
#         merges them using Tukey loss (strict) to produce a local submap and pushes it into
#         self.merged_buffer for global alignment.
#         """
#         self.get_logger().info("Local ICP (merge) thread started.")

#         def make_local_loss(scale):
#             return o3d.pipelines.registration.HuberLoss(scale)

#         def estimate_overlap(first, last, radius):
#             """Rough fraction of points in `first` that have a neighbor in `last` within `radius`.
#             We use small voxel-downsampled clouds for speed."""
#             try:
#                 a = first.voxel_down_sample(self.icp_voxel_size)
#                 b = last.voxel_down_sample(self.icp_voxel_size)
#                 if len(a.points) == 0 or len(b.points) == 0:
#                     return 0.0
#                 kdt = o3d.geometry.KDTreeFlann(b)
#                 pts_a = np.asarray(a.points)
#                 close = 0
#                 # sample if very dense to keep runtime bounded
#                 max_checks = 512
#                 step = max(1, pts_a.shape[0] // max_checks)
#                 for p in pts_a[::step]:
#                     _k, idx, _ = kdt.search_knn_vector_3d(p, 1)
#                     if _k > 0:
#                         bpts = np.asarray(b.points)
#                         if np.linalg.norm(p - bpts[idx[0]]) <= radius:
#                             close += 1
#                 total = max(1, int(np.ceil(float(pts_a.shape[0]) / step)))
#                 return float(close) / float(total)
#             except Exception:
#                 return 0.0

#         while not self.shutdown_event.is_set():
#             # Wait until a new frame arrives or timeout
#             self.new_frame_event.wait(self.icp_run_interval)
#             self.new_frame_event.clear()
#             if self.shutdown_event.is_set():
#                 break

#             # snapshot buffer
#             with self.buffer_lock:
#                 buffer_snapshot = [pcd.clone() if hasattr(pcd, "clone") else deepcopy(pcd)
#                                    for pcd in self.pcd_buffer]

#             if len(buffer_snapshot) == 0:
#                 continue

#             # adaptive submap sizing: if first/last overlap is low, create smaller submaps
#             try:
#                 overlap_radius = max(1e-3, self.icp_max_corr * 1.5)
#                 overlap_est = estimate_overlap(buffer_snapshot[0], buffer_snapshot[-1], overlap_radius)
#             except Exception:
#                 overlap_est = 1.0

#             if overlap_est < self.icp_overlap_threshold:
#                 effective_submap_size = max(1, int(self.icp_submap_size * self.icp_adaptive_submap_factor))
#             else:
#                 effective_submap_size = self.icp_submap_size

#             # Decide whether to flush a submap based on hybrid rule (N frames or T seconds)
#             now = time.time()
#             if len(buffer_snapshot) < effective_submap_size and (now - self._last_submap_time) < self.icp_submap_time:
#                 # Not enough frames yet and not timed out -> continue waiting
#                 continue

#             try:
#                 # --- Merge buffer_snapshot into a local "merged" submap using Tukey ---
#                 if len(buffer_snapshot) == 1:
#                     merged = buffer_snapshot[0]
#                 else:
#                     target = buffer_snapshot[0]
#                     target_down = target.voxel_down_sample(self.icp_voxel_size)
#                     target_down.estimate_normals(
#                         o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30)
#                     )

#                     merged = target_down
#                     for i in range(1, len(buffer_snapshot)):
#                         source = buffer_snapshot[i]
#                         source_down = source.voxel_down_sample(self.icp_voxel_size)
#                         source_down.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30)
#                         )

#                         reg = o3d.pipelines.registration.registration_icp(
#                             source_down, merged,
#                             self.icp_max_corr,
#                             np.eye(4),
#                             o3d.pipelines.registration.TransformationEstimationPointToPlane(
#                                 make_local_loss(self.icp_voxel_size))
#                         )
#                         T = reg.transformation
#                         source_transformed = deepcopy(source).transform(T)
#                         merged += source_transformed
#                         merged = merged.voxel_down_sample(self.icp_downsample_voxel)
#                         merged.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_downsample_voxel*2, max_nn=30)
#                         )

#                 # --- Normalize density to avoid overweighting close, dense regions ---
#                 try:
#                     merged = self._normalize_density(merged)
#                 except Exception as e:
#                     self.get_logger().warn(f"Density normalization failed: {e}")

#                 # push merged into merged_buffer (store as tuple (pcd, attempts))
#                 try:
#                     merged_copy = merged.clone() if hasattr(merged, "clone") else deepcopy(merged)
#                 except Exception:
#                     merged_copy = deepcopy(merged)

#                 with self.merged_lock:
#                     self.merged_buffer.append((merged_copy, 0))
#                     # signal the global worker
#                     self.new_merged_event.set()

#                 # drop the frames we just merged from the pcd_buffer to avoid reprocessing them
#                 with self.buffer_lock:
#                     # remove up to len(buffer_snapshot) oldest entries (if they still exist)
#                     n_drop = min(len(buffer_snapshot), len(self.pcd_buffer))
#                     if n_drop > 0:
#                         self.pcd_buffer = self.pcd_buffer[n_drop:]

#                 self._last_submap_time = time.time()
#                 self.get_logger().info(f"Local merge: produced submap from {len(buffer_snapshot)} frames -> merged_buffer size {len(self.merged_buffer)}.")
#             except Exception as e:
#                 self.get_logger().warn(f"Local merge worker failed: {e}")
#                 time.sleep(0.05)

#         self.get_logger().info("Local merge thread exiting.")


#     def _global_align_worker(self):
#         """
#         Consumer: waits for merged submaps in self.merged_buffer, aligns each submap to
#         the global_pcd using multi-scale ICP with Huber loss, then updates global_pcd.

#         This variant:
#           - uses a single start (last_global_transform)
#           - adaptive max correspondence per scale
#           - accept/reject with requeue attempts
#           - stores accepted submaps into self.submap_graph for later refinement
#         """
#         self.get_logger().info("Global ICP (align) thread started.")
#         last_global_transform = np.eye(4)

#         def make_global_loss(scale):
#             return o3d.pipelines.registration.TukeyLoss(scale)

#         while not self.shutdown_event.is_set():
#             # Wait for a new merged submap or timeout (so we can exit cleanly)
#             self.new_merged_event.wait(self.icp_run_interval)
#             self.new_merged_event.clear()
#             if self.shutdown_event.is_set():
#                 break

#             # pop one merged submap if available
#             with self.merged_lock:
#                 if len(self.merged_buffer) == 0:
#                     continue
#                 item = self.merged_buffer.pop(0)

#             # item may be (pcd, attempts)
#             if isinstance(item, tuple) and len(item) == 2:
#                 merged, attempts = item
#             else:
#                 merged = item
#                 attempts = 0

#             try:
#                 # keep an original copy for requeue/integration
#                 orig_merged = merged.clone() if hasattr(merged, "clone") else deepcopy(merged)

#                 # prepare a (downsampled) copy of the global map for fast ICP
#                 with self.buffer_lock:
#                     global_copy = None
#                     if len(self.global_pcd.points) > 0:
#                         global_copy = self.global_pcd.voxel_down_sample(self.icp_voxel_size)
#                         global_copy.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_voxel_size*2, max_nn=30)
#                         )

#                 if global_copy is not None:
#                     # multi-scale ICP (coarse -> fine) using Huber loss
#                     voxel_radii = [self.icp_voxel_size*4, self.icp_voxel_size*2, self.icp_voxel_size]
#                     max_iters = [50, 30, 14]

#                     current_trans = last_global_transform
#                     reg = None
#                     for scale, max_iter in zip(voxel_radii, max_iters):
#                         src_down = merged.voxel_down_sample(scale)
#                         tgt_down = global_copy.voxel_down_sample(scale)
#                         src_down.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=scale*2, max_nn=30))
#                         tgt_down.estimate_normals(
#                             o3d.geometry.KDTreeSearchParamHybrid(radius=scale*2, max_nn=30))

#                         # adaptive max correspondence distance (scale-sensitive)
#                         base_corr = max(1e-6, float(self.icp_max_corr))
#                         scale_multiplier = max(1.0, scale / max(1e-6, float(self.icp_voxel_size)))
#                         corr = base_corr * scale_multiplier * float(self.icp_max_corr_scale)

#                         reg = o3d.pipelines.registration.registration_icp(
#                             src_down, tgt_down,
#                             corr,
#                             current_trans,
#                             o3d.pipelines.registration.TransformationEstimationPointToPlane(
#                                 make_global_loss(scale)),
#                             o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
#                         )
#                         current_trans = reg.transformation

#                     # final registration result is in `reg` (from last scale)
#                     best_fitness = reg.fitness if reg is not None else 0.0
#                     best_rmse = reg.inlier_rmse if reg is not None else float('inf')

#                     # acceptance test
#                     if reg is None or best_fitness < float(self.icp_accept_fitness) or best_rmse > float(self.icp_max_rmse):
#                         attempts += 1
#                         if attempts <= int(self.icp_max_attempts):
#                             with self.merged_lock:
#                                 self.merged_buffer.append((orig_merged, attempts))
#                             self.get_logger().warn(f"Global align: rejected submap (fitness={best_fitness:.3f}, rmse={best_rmse:.4f}), requeued (attempt {attempts}).")
#                             continue
#                         else:
#                             self.get_logger().warn(f"Global align: dropped submap after {attempts} attempts (fitness={best_fitness:.3f}, rmse={best_rmse:.4f}).")
#                             continue

#                     # accepted: apply transform
#                     orig_merged.transform(current_trans)
#                     last_global_transform = current_trans
#                 else:
#                     # empty global -> insert first submap as-is
#                     self.get_logger().info("Global align: empty global map, inserting first submap.")
#                     # no ICP, keep orig_merged coordinates
#                     last_global_transform = last_global_transform

#                 # bookkeeping: keep submaps separately (for later refinement)
#                 try:
#                     submap_entry = {'pcd': orig_merged.clone() if hasattr(orig_merged, 'clone') else deepcopy(orig_merged),
#                                     'pose': last_global_transform}
#                 except Exception:
#                     submap_entry = {'pcd': deepcopy(orig_merged), 'pose': last_global_transform}

#                 self.submap_graph.append(submap_entry)

#                 with self.buffer_lock:
#                     if self.use_submap_graph_mode:
#                         # graph-only: defer fusion
#                         pass
#                     else:
#                         if len(self.global_pcd.points) == 0:
#                             self.global_pcd = orig_merged
#                         else:
#                             self.global_pcd += orig_merged
#                             try:
#                                 self.global_pcd = self.global_pcd.voxel_down_sample(self.icp_downsample_voxel)
#                             except Exception:
#                                 pass

#                 self.get_logger().info(f"Global align: integrated submap (merged_buffer size {len(self.merged_buffer)}) -> global points {len(self.global_pcd.points)}.")
#             except Exception as e:
#                 self.get_logger().warn(f"Global align worker failed: {e}")
#                 time.sleep(0.1)

#         self.get_logger().info("Global align thread exiting.")


#     def save_results(self):
#         timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
#         out_dir = f"tsdf_output_{timestamp}"
#         os.makedirs(out_dir, exist_ok=True)

#         # Save final global_pcd
#         o3d.io.write_point_cloud(os.path.join(out_dir, "global_pcd.ply"), self.global_pcd)

#         # Save poses
#         with open(os.path.join(out_dir, "poses.csv"), "w") as f:
#             writer = csv.writer(f)
#             writer.writerow(["frame_id", "timestamp"] + [f"T{i}{j}" for i in range(4) for j in range(4)])
#             writer.writerows(self.pose_records)

#         if rclpy.ok():
#             self.get_logger().info(f"Saved results to {out_dir}")



# def main(args=None):
#     rclpy.init(args=args)
#     node = TSDFFusionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # signal ICP worker to exit and wait for it
#         try:
#             node.shutdown_event.set()
#             node.new_frame_event.set()  # wake the thread if it's waiting
#             node.new_merged_event.set()
#             if node.icp_thread.is_alive():
#                 node.get_logger().info("Waiting for ICP thread to finish...")
#                 node.icp_thread.join(timeout=10.0)
#             if node.global_thread.is_alive():
#                 node.get_logger().info("Waiting for global ICP thread to finish...")
#                 node.global_thread.join(timeout=10.0)
#         except Exception:
#             pass

#     # Run final color ICP and save results **before ROS shutdown**
#     node.save_results()

#     # Now destroy node and shutdown ROS
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
