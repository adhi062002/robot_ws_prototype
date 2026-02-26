# import open3d as o3d
# import rclpy
# from rclpy.node import Node
# import message_filters
# from sensor_msgs.msg import Image
# from nav_msgs.msg import Odometry
# from cv_bridge import CvBridge
# import numpy as np
# import cv2


# class ICPPrototype(Node):
#     def __init__(self):
#         super().__init__('open3d_icp_prototype')

#         self.bridge = CvBridge()
#         self.rgb_sub = message_filters.Subscriber(self, Image, '/ascamera_hp60c/camera_publisher/rgb0/image')
#         self.depth_sub = message_filters.Subscriber(self, Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw')
#         self.odom_sub = message_filters.Subscriber(self, Odometry, '/odometry_rect')

#         self.ts = message_filters.ApproximateTimeSynchronizer(
#             [self.rgb_sub, self.depth_sub, self.odom_sub],
#             queue_size=10,
#             slop=0.05
#         )
#         self.ts.registerCallback(self.sync_callback)

#         # Camera intrinsics
#         fx, fy, cx, cy = 525.0, 525.0, 319.5, 239.5
#         width, height = 640, 480
#         self.intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

#         # Camera to body transformation
#         self.T_body_cam = np.array([
#             [-0.02156132, -0.99483675, -0.09917134, 0.03432751],
#             [-0.25782609, 0.10137213, -0.96085868, 0.03072772],
#             [0.96595073, 0.00485158, -0.25868058, -0.07285357],
#             [0., 0., 0., 1.]
#         ])

#         self.frames = []  # store (rgbd, odom)
#         self.max_frames = 150
#         self.depth_trunc = 2.0
#         self.get_logger().info("Initialized ICP Prototype Node")

#     # -------------------------------------------------------
#     # Utility functions
#     # -------------------------------------------------------
#     def odom_to_matrix(self, odom: Odometry):
#         """Convert ROS Odometry to 4x4 transformation matrix."""
#         pose = odom.pose.pose
#         tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
#         qx, qy, qz, qw = (
#             pose.orientation.x,
#             pose.orientation.y,
#             pose.orientation.z,
#             pose.orientation.w,
#         )

#         # Normalize quaternion
#         norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
#         qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

#         R = np.array([
#             [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
#             [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
#             [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
#         ])
#         T = np.eye(4)
#         T[:3, :3] = R
#         T[:3, 3] = [tx, ty, tz]
#         return T

#     # -------------------------------------------------------
#     def sync_callback(self, rgb_msg: Image, depth_msg: Image, odom_msg: Odometry):
#         if len(self.frames) >= self.max_frames:
#             return

#         rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
#         depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

#         depth_np = np.asarray(depth_cv, dtype=np.float32)
#         depth_scale = 1000.0  # mm → meters

#         color_o3d = o3d.geometry.Image(cv2.cvtColor(rgb_cv, cv2.COLOR_BGR2RGB))
#         depth_o3d = o3d.geometry.Image(depth_np)
#         rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
#             color_o3d, depth_o3d,
#             depth_scale=depth_scale,
#             depth_trunc=self.depth_trunc,
#             convert_rgb_to_intensity=False
#         )

#         self.frames.append((rgbd, odom_msg))
#         self.get_logger().info(f"Captured frame {len(self.frames)}")

#         if len(self.frames) == self.max_frames:
#             self.get_logger().info("Collected frames. Computing odometry comparison...")
#             self.compare_odometry()

#     # -------------------------------------------------------
#     def compare_odometry(self):
#         o3d_poses = [np.eye(4)]
#         ros_poses = [np.eye(4)]

#         for i in range(1, len(self.frames)):
#             source_rgbd, source_odom = self.frames[i-1]
#             target_rgbd, target_odom = self.frames[i]

#             # Compute Open3D RGB-D odometry
#             option = o3d.pipelines.odometry.OdometryOption()
#             odo_init = np.identity(4)

#             success, trans, _ = o3d.pipelines.odometry.compute_rgbd_odometry(
#                 source_rgbd,
#                 target_rgbd,
#                 self.intrinsics,
#                 odo_init,
#                 o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(),
#                 option
#             )

#             if success:
#                 o3d_poses.append(o3d_poses[-1] @ trans)

#             # ROS /odom/rect pose
#             ros_T_prev = self.odom_to_matrix(source_odom)
#             ros_T_curr = self.odom_to_matrix(target_odom)
#             relative_ros = np.linalg.inv(ros_T_prev) @ ros_T_curr
#             ros_poses.append(ros_poses[-1] @ relative_ros)

#         # ---------------------------------------------------
#         # Visualize results
#         # ---------------------------------------------------
#         pcs_open3d = []
#         pcs_ros = []

#         for i, (rgbd, _) in enumerate(self.frames):
#             # Open3D odometry pose
#             pcd_open3d = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)
#             pcd_open3d.transform(o3d_poses[i])
#             pcs_open3d.append(pcd_open3d)

#             # ROS odometry pose
#             pcd_ros = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)
#             pcd_ros.transform(ros_poses[i])
#             pcs_ros.append(pcd_ros)

#         self.get_logger().info("Showing Open3D odometry reconstruction...")
#         o3d.visualization.draw_geometries(pcs_open3d, window_name="Open3D RGBD Odometry")

#         self.get_logger().info("Showing ROS /odom/rect reconstruction...")
#         o3d.visualization.draw_geometries(pcs_ros, window_name="ROS Odometry")

#         self.get_logger().info("Done.")
#         rclpy.shutdown()


# # -------------------------------------------------------
# def main(args=None):
#     rclpy.init(args=args)
#     node = ICPPrototype()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
# import open3d as o3d
# import rclpy
# from rclpy.node import Node
# import message_filters
# from sensor_msgs.msg import Image
# from nav_msgs.msg import Odometry
# from cv_bridge import CvBridge
# import numpy as np
# import cv2

# class ICPPrototype(Node):
#     def __init__(self):
#         super().__init__('open3d_prototype')

#         self.bridge = CvBridge()

#         self.rgb_topic = message_filters.Subscriber(self, Image, '/ascamera_hp60c/camera_publisher/rgb0/image')
#         self.depth_topic = message_filters.Subscriber(self, Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw')
#         self.odom_sub = message_filters.Subscriber(self, Odometry, '/odometry_rect')

#         self.ts = message_filters.ApproximateTimeSynchronizer(
#             [self.rgb_topic, self.depth_topic, self.odom_sub],
#             queue_size=10, slop=0.01
#         )
#         self.ts.registerCallback(self.sync_callback)

#         self.T_body_cam = np.array([
#             [-0.02156132, -0.99483675, -0.09917134,  0.03432751],
#             [-0.25782609,  0.10137213, -0.96085868,  0.03072772],
#             [ 0.96595073,  0.00485158, -0.25868058, -0.07285357],
#             [ 0.,           0.,           0.,           1.       ]
#         ])

#         fx, fy, cx, cy = 525.0, 525.0, 319.5, 239.5
#         width, height = 640, 480
#         self.intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
#         self.depth_trunc = 2.0

#         self.rgbd_frames = []
#         self.odom_frames = []
#         self.max_frames = 150

#         self.get_logger().info("Initialized ICPPrototype Node")

#     def odom_to_matrix(self, odom: Odometry):
#         pose = odom.pose.pose
#         tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
#         qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

#         norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
#         qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

#         R = np.array([
#             [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
#             [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
#             [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]
#         ], dtype=np.float64)
#         T = np.eye(4)
#         T[:3, :3] = R
#         T[:3, 3] = np.array([tx, ty, tz])
#         return T

#     def _depth_msg_to_numpy(self, depth_msg, depth_cv_raw):
#         depth_np = np.asarray(depth_cv_raw, dtype=np.float32)
#         depth_scale = 1000.0
#         return depth_np, depth_scale

#     def sync_callback(self, rgb_msg: Image, depth_msg: Image, odom_msg: Odometry):
#         if len(self.rgbd_frames) >= self.max_frames:
#             return

#         rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
#         depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

#         depth_np, depth_scale = self._depth_msg_to_numpy(depth_msg, depth_cv_raw)

#         color_o3d = o3d.geometry.Image(cv2.cvtColor(np.ascontiguousarray(rgb_cv), cv2.COLOR_BGR2RGB))
#         depth_o3d = o3d.geometry.Image(np.ascontiguousarray(depth_np))

#         rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
#             color_o3d, depth_o3d,
#             depth_scale=depth_scale,
#             depth_trunc=self.depth_trunc,
#             convert_rgb_to_intensity=False
#         )

#         self.rgbd_frames.append(rgbd)
#         self.odom_frames.append(odom_msg)

#         self.get_logger().info(f"Captured frame {len(self.rgbd_frames)}/{self.max_frames}")

#         if len(self.rgbd_frames) == self.max_frames:
#             self.get_logger().info("Collected all frames. Starting odometry comparison...")
#             self.compare_odometry()

#     def compare_odometry(self):
#         o3d_poses = [np.eye(4)]
#         ros_poses = [np.eye(4)]

#         option = o3d.pipelines.odometry.OdometryOption()
#         odo_init = np.identity(4)

#         for i in range(1, len(self.rgbd_frames)):
#             source_rgbd = self.rgbd_frames[i - 1]
#             target_rgbd = self.rgbd_frames[i]
#             source_odom = self.odom_frames[i - 1]
#             target_odom = self.odom_frames[i]

#             [success, trans, info] = o3d.pipelines.odometry.compute_rgbd_odometry(
#                 source_rgbd, target_rgbd,
#                 self.intrinsics, odo_init,
#                 o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(),
#                 option
#             )

#             if success:
#                 o3d_poses.append(o3d_poses[-1] @ trans)
#             else:
#                 o3d_poses.append(o3d_poses[-1])

#             T_prev = self.odom_to_matrix(source_odom)
#             T_curr = self.odom_to_matrix(target_odom)
#             relative_ros = np.linalg.inv(T_prev) @ T_curr
#             ros_poses.append(ros_poses[-1] @ relative_ros)

#         self.get_logger().info("Creating point clouds for visualization...")

#         pcs_o3d = []
#         pcs_ros = []

#         for i, rgbd in enumerate(self.rgbd_frames):
#             pcd_o3d = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)
#             pcd_o3d.transform(o3d_poses[i])
#             pcs_o3d.append(pcd_o3d)

#             pcd_ros = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)
#             pcd_ros.transform(ros_poses[i])
#             pcs_ros.append(pcd_ros)

#         self.get_logger().info("Displaying Open3D RGB-D odometry reconstruction...")
#         o3d.visualization.draw_geometries(pcs_o3d, window_name="Open3D Odometry Reconstruction")

#         self.get_logger().info("Displaying ROS /odom/rect reconstruction...")
#         o3d.visualization.draw_geometries(pcs_ros, window_name="ROS Odometry Reconstruction")

#         self.get_logger().info("Done.")
#         rclpy.shutdown()


# def main(args=None):
#     rclpy.init(args=args)
#     node = ICPPrototype()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()
import open3d as o3d
import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import cv2
import copy


class ICPPrototype(Node):
    def __init__(self):
        super().__init__('open3d_prototype')

        # --- ROS / cv bridge ---
        self.bridge = CvBridge()
        self.rgb_topic = message_filters.Subscriber(self, Image, '/ascamera_hp60c/camera_publisher/rgb0/image')
        self.depth_topic = message_filters.Subscriber(self, Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw')
        self.odom_sub = message_filters.Subscriber(self, Odometry, '/odometry_rect')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_topic, self.depth_topic, self.odom_sub],
            queue_size=10, slop=0.01
        )
        self.ts.registerCallback(self.sync_callback)

        # --- Camera extrinsics (body->cam) and intrinsics ---
        self.T_body_cam = np.array([
            [-0.02156132, -0.99483675, -0.09917134,  0.03432751],
            [-0.25782609,  0.10137213, -0.96085868,  0.03072772],
            [ 0.96595073,  0.00485158, -0.25868058, -0.07285357],
            [ 0.,           0.,           0.,           1.       ]
        ])

        fx, fy, cx, cy = 525.0, 525.0, 319.5, 239.5
        width, height = 640, 480
        self.intrinsics = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
        self.depth_trunc = 4.0

        # --- Collect frames ---
        self.rgbd_frames = []        # list of o3d.geometry.RGBDImage
        self.odom_frames = []        # list of nav_msgs/Odometry
        self.max_frames = 150        # as requested

        # --- Multiway / ICP params ---
        self.voxel_size = 0.02                       # requested
        self.max_correspondence_distance_coarse = 0.05
        self.max_correspondence_distance_fine = 0.02
        self.loop_closure_interval = 15              # sparse loop closures to keep computation manageable

        self.get_logger().info("Initialized ICPPrototype Node")

    # -------------------------------------------------------
    def odom_to_matrix(self, odom: Odometry):
        pose = odom.pose.pose
        tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

        norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm == 0:
            norm = 1.0
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
        ], dtype=np.float64)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.array([tx, ty, tz])
        return T

    # -------------------------------------------------------
    def _depth_msg_to_numpy(self, depth_msg, depth_cv_raw):
        # Keep raw values (often uint16 millimeters). We convert to float32 and keep mm units;
        # create_from_color_and_depth will divide by depth_scale to get meters.
        depth_np = np.asarray(depth_cv_raw, dtype=np.float32)
        depth_scale = 1000.0
        return depth_np, depth_scale

    # -------------------------------------------------------
    def sync_callback(self, rgb_msg: Image, depth_msg: Image, odom_msg: Odometry):
        if len(self.rgbd_frames) >= self.max_frames:
            return

        rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth_cv_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        depth_np, depth_scale = self._depth_msg_to_numpy(depth_msg, depth_cv_raw)

        color_o3d = o3d.geometry.Image(cv2.cvtColor(np.ascontiguousarray(rgb_cv), cv2.COLOR_BGR2RGB))
        depth_o3d = o3d.geometry.Image(np.ascontiguousarray(depth_np))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color_o3d, depth_o3d,
            depth_scale=depth_scale,
            depth_trunc=self.depth_trunc,
            convert_rgb_to_intensity=False
        )

        # store rgbd and the odom message
        self.rgbd_frames.append(rgbd)
        self.odom_frames.append(odom_msg)

        self.get_logger().info(f"Captured frame {len(self.rgbd_frames)}/{self.max_frames}")

        if len(self.rgbd_frames) == self.max_frames:
            self.get_logger().info("Collected all frames. Starting odometry comparison and multiway registration...")
            # run heavy processing in-line (user asked for it)
            self.process_all()

    # -------------------------------------------------------
    def process_all(self):
        # 1) compute trajectories: Open3D odometry (chained) and ROS odometry (chained relative)
        # o3d_poses = [np.eye(4)]
        ros_poses = [np.eye(4)]

        # option = o3d.pipelines.odometry.OdometryOption()
        # odo_init = np.identity(4)

        # self.get_logger().info("Computing frame-to-frame Open3D RGBD odometry (sequential)...")
        # for i in range(1, len(self.rgbd_frames)):
        #     source_rgbd = self.rgbd_frames[i - 1]
        #     target_rgbd = self.rgbd_frames[i]

        #     success, trans, info = o3d.pipelines.odometry.compute_rgbd_odometry(
        #         source_rgbd, target_rgbd, self.intrinsics, odo_init,
        #         o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(), option
        #     )

        #     if success:
        #         o3d_poses.append(o3d_poses[-1] @ trans)
        #     else:
        #         # if odometry fails, keep the previous pose (no change)
        #         o3d_poses.append(o3d_poses[-1])
        #     if i % 20 == 0:
        #         self.get_logger().info(f"Processed Open3D odometry for frame {i}/{len(self.rgbd_frames)-1}")

        self.get_logger().info("Computing ROS relative odometry chain...")
        for i in range(1, len(self.odom_frames)):
            T_prev = self.odom_to_matrix(self.odom_frames[i - 1])
            T_curr = self.odom_to_matrix(self.odom_frames[i])
            # relative transform from prev->curr in world coordinates
            relative_ros = np.linalg.inv(T_prev) @ T_curr
            ros_poses.append(ros_poses[-1] @ relative_ros)

        # 2) build pointclouds for both trajectories (and downsample)
        self.get_logger().info("Building pointclouds and downsampling (voxel_size = %.3f)..." % self.voxel_size)

        # pcds_o3d = []
        pcds_ros = []

        for i, rgbd in enumerate(self.rgbd_frames):
            # create pointcloud in camera frame
            pcd_cam = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsics)

            # 2a) Open3D odometry placement
            # pcd_o = copy.deepcopy(pcd_cam)
            # pcd_o.transform(o3d_poses[i])
            # pcd_o_down = pcd_o.voxel_down_sample(voxel_size=self.voxel_size)
            # pcd_o_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            #     radius=self.voxel_size * 2.0, max_nn=30))
            # pcds_o3d.append(pcd_o_down)

            # 2b) ROS odometry placement
            pcd_r = copy.deepcopy(pcd_cam)
            # Note: if you want to apply body->cam extrinsic here, you can (T_w_cam = T_w_body @ T_body_cam).
            # For a fair comparison we used camera-frame-based chained transforms, consistent with o3d_poses
            pcd_r.transform(ros_poses[i])
            pcd_r_down = pcd_r.voxel_down_sample(voxel_size=self.voxel_size)
            pcd_r_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 2.0, max_nn=30))
            pcds_ros.append(pcd_r_down)

            if (i + 1) % 50 == 0:
                self.get_logger().info(f"Prepared downsampled pcds for frame {i+1}/{len(self.rgbd_frames)}")

        # 3) Multiway registration (pose graph) on both sets
        # self.get_logger().info("Running multiway (pose-graph) registration on Open3D-odometry-aligned clouds...")
        # pose_graph_o3d = self.full_registration(pcds_o3d,
        #                                         self.max_correspondence_distance_coarse,
        #                                         self.max_correspondence_distance_fine,
        #                                         loop_closure_interval=self.loop_closure_interval)
        # # optimize
        # self.run_global_optimization(pose_graph_o3d)

        # apply optimized poses and combine
        # combined_o3d = self.combine_registered(pcds_o3d, pose_graph_o3d, "multiway_open3d.pcd")

        self.get_logger().info("Running multiway (pose-graph) registration on ROS-odometry-aligned clouds...")
        pose_graph_ros = self.full_registration(pcds_ros,
                                                self.max_correspondence_distance_coarse,
                                                self.max_correspondence_distance_fine,
                                                loop_closure_interval=self.loop_closure_interval)
        self.run_global_optimization(pose_graph_ros)
        combined_ros = self.combine_registered(pcds_ros, pose_graph_ros, "multiway_ros.pcd")

        # 4) visualization: show both combined results (separately)
        self.get_logger().info("Visualizing results...")
        # o3d.visualization.draw_geometries([combined_o3d], window_name="Multiway (Open3D odometry)")
        o3d.visualization.draw_geometries([combined_ros], window_name="Multiway (ROS odometry)")

        self.get_logger().info("Completed multiway registration for both trajectories. Shutting down rclpy.")
        rclpy.shutdown()

    # # -------------------------------------------------------
    # pairwise and full registration (adapted from your snippet)
    def pairwise_registration(self, source, target):
        print("Apply point-to-plane ICP")
        icp_coarse = o3d.pipelines.registration.registration_icp(
            source, target,
            self.max_correspondence_distance_coarse,
            np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        icp_fine = o3d.pipelines.registration.registration_icp(
            source, target,
            self.max_correspondence_distance_fine,
            icp_coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane()
        )
        transformation_icp = icp_fine.transformation
        information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            source, target, self.max_correspondence_distance_fine,
            icp_fine.transformation
        )
        return transformation_icp, information_icp

    def full_registration(self, pcds, max_correspondence_distance_coarse,
                          max_correspondence_distance_fine, loop_closure_interval=20):
        """
        Build a PoseGraph with odometry edges between consecutive frames and
        sparse loop-closure edges every `loop_closure_interval` frames.
        (All-vs-all is too expensive for large N; we therefore use a sparse strategy.)
        """
        pose_graph = o3d.pipelines.registration.PoseGraph()
        odometry = np.identity(4)
        pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
        n_pcds = len(pcds)

        # consecutive (odometry) edges
        for source_id in range(n_pcds - 1):
            target_id = source_id + 1
            transformation_icp, information_icp = self.pairwise_registration(
                pcds[source_id], pcds[target_id])
            print("Build o3d.pipelines.registration.PoseGraph (odometry edge)")
            odometry = np.dot(transformation_icp, odometry)
            pose_graph.nodes.append(
                o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry)))
            pose_graph.edges.append(
                o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                         target_id,
                                                         transformation_icp,
                                                         information_icp,
                                                         uncertain=False)
            )

        # sparse loop-closure edges (every interval)
        for source_id in range(n_pcds):
            for target_id in range(source_id + loop_closure_interval, n_pcds, loop_closure_interval):
                transformation_icp, information_icp = self.pairwise_registration(
                    pcds[source_id], pcds[target_id])
                print("Build o3d.pipelines.registration.PoseGraph (loop edge)")
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True)
                )
        return pose_graph

    def run_global_optimization(self, pose_graph):
        print("Optimizing PoseGraph ...")
        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=self.max_correspondence_distance_fine,
            edge_prune_threshold=0.25,
            reference_node=0
        )
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            o3d.pipelines.registration.global_optimization(
                pose_graph,
                o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
                o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
                option
            )

    def combine_registered(self, pcds, pose_graph, out_filename):
        # apply optimized poses to pcds and combine
        pcd_combined = o3d.geometry.PointCloud()
        for point_id in range(len(pcds)):
            pose = pose_graph.nodes[point_id].pose
            pcd = copy.deepcopy(pcds[point_id])
            pcd.transform(pose)
            pcd_combined += pcd
        pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=self.voxel_size)
        o3d.io.write_point_cloud(out_filename, pcd_combined_down)
        print(f"Saved combined pointcloud to {out_filename}")
        return pcd_combined_down


# -------------------------------------------------------
def main(args=None):        
    rclpy.init(args=args)
    node = ICPPrototype()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # cleanup in case of early exit
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
