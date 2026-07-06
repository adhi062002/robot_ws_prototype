#!/usr/bin/env python3
"""
dent_mapping_node.py  ─  ROS2 Humble
══════════════════════════════════════════════════════════════════════════════
Pipeline
────────
  RGB stream  ──► YOLO (dent detector)
                    │
                    ├─ bbox present?  ──► mask depth ROI ──► backproject
                    │                         │                  │
                    │                   3-D dent patch ──► transform to map
                    │                                           │
                    │                               freeze into dent_cloud  ◄─ PERMANENT
                    │
  RGB-D pair  ──►  SuperPoint + SuperGlue
                    │
                    ▼
              TEASER++ (robust 3-D reg)
                    │
                    ▼
              Colored ICP (fine)
                    │
                    ▼
              T_global ──► accumulate scene_combined

Published topics
────────────────
  /dent_mapping/scene_cloud        PointCloud2   full accumulated scene
  /dent_mapping/dent_cloud         PointCloud2   frozen dent patches (map frame)
  /dent_mapping/detection_image    Image         RGB with YOLO bboxes drawn
  /dent_mapping/overlay_image      Image         dent bbox region composited
  /tf                              TF            frame_N-1→frame_N, map→camera

Subscribed (live mode)
──────────────────────
  /camera/color/image_raw
  /camera/depth/image_rect_raw
  /camera/color/camera_info

Parameters
──────────
  source              "live" | "bag"
  yolo_model_path     path to .pt weights  (default: yolov8n.pt auto-download)
  yolo_conf           float confidence threshold  default 0.4
  yolo_class_ids      int[]  YOLO class ids to treat as dents ([] = all)
  dent_min_depth      float  ignore depth < this (m)  default 0.1
  dent_max_depth      float  ignore depth > this (m)  default 3.5
  voxel_size          float  default 0.01
  bag_rgb_paths       str[]
  bag_depth_paths     str[]
  map_frame           default "map"
  camera_frame        default "camera_color_optical_frame"
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import numpy as np
import cv2
import copy
import torch
import open3d as o3d
import teaserpp_python

from pathlib import Path
from threading import Lock
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import tf2_ros
from scipy.spatial.transform import Rotation

# SuperGlue / SuperPoint  (PYTHONPATH must include SuperGluePretrainedNetwork/)
from models.matching import Matching
from models.utils import read_image

# Ultralytics YOLO
from ultralytics import YOLO


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

FLIP = np.array([[1, 0, 0, 0],
                 [0,-1, 0, 0],
                 [0, 0,-1, 0],
                 [0, 0, 0, 1]], dtype=np.float64)

# Colour painted on frozen dent points (bright red in Open3D RGB)
DENT_COLOUR = [1.0, 0.15, 0.1]


# ─────────────────────────────────────────────────────────────────────────────
# Dataclass: one frozen dent patch
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class DentPatch:
    frame_idx:  int
    confidence: float
    class_id:   int
    bbox_xyxy:  Tuple[int, int, int, int]   # pixel coords at detection time
    pcd:        o3d.geometry.PointCloud      # already in map frame, frozen


# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

def _make_header(frame_id: str, stamp) -> Header:
    h = Header()
    h.stamp    = stamp
    h.frame_id = frame_id
    return h


def o3d_to_pc2(pcd: o3d.geometry.PointCloud,
               frame_id: str, stamp) -> PointCloud2:
    pts = np.asarray(pcd.points, dtype=np.float32)
    if pts.shape[0] == 0:
        # Publish an empty cloud rather than crash
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        return pc2.create_cloud(_make_header(frame_id, stamp), fields, [])

    fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    return pc2.create_cloud(_make_header(frame_id, stamp), fields,
                            pts.tolist())


def mat_to_ts(T: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    ts = TransformStamped()
    ts.header.stamp    = stamp
    ts.header.frame_id = parent
    ts.child_frame_id  = child
    ts.transform.translation.x = float(T[0, 3])
    ts.transform.translation.y = float(T[1, 3])
    ts.transform.translation.z = float(T[2, 3])
    q = Rotation.from_matrix(T[:3, :3]).as_quat()   # xyzw
    ts.transform.rotation.x = float(q[0])
    ts.transform.rotation.y = float(q[1])
    ts.transform.rotation.z = float(q[2])
    ts.transform.rotation.w = float(q[3])
    return ts


def backproject_mask(mask2d: np.ndarray, depth: np.ndarray,
                     fx, fy, cx, cy,
                     d_min: float = 0.1,
                     d_max: float = 3.5) -> np.ndarray:
    """
    Dense backprojection of all pixels inside a 2-D boolean mask.
    Returns Nx3 array of valid 3-D points in camera frame.
    """
    ys, xs = np.where(mask2d)
    if len(xs) == 0:
        return np.zeros((0, 3), dtype=np.float64)
    z = depth[ys, xs]
    valid = (z > d_min) & (z < d_max) & np.isfinite(z)
    xs, ys, z = xs[valid], ys[valid], z[valid]
    x = (xs - cx) * z / fx
    y = (ys - cy) * z / fy
    return np.vstack((x, y, z)).T


def backproject_kpts(kpts: np.ndarray, depth: np.ndarray,
                     fx, fy, cx, cy):
    """Sparse backprojection for SuperGlue keypoints."""
    r = np.clip(kpts[:, 1].astype(int), 0, depth.shape[0] - 1)
    c = np.clip(kpts[:, 0].astype(int), 0, depth.shape[1] - 1)
    z = depth[r, c]
    x = (kpts[:, 0] - cx) * z / fx
    y = (kpts[:, 1] - cy) * z / fy
    pts = np.vstack((x, y, z)).T
    valid = (z > 0) & np.isfinite(z)
    return pts[valid], valid


def make_rgbd_pcd(rgb_bgr: np.ndarray, depth_m: np.ndarray,
                  intr: o3d.camera.PinholeCameraIntrinsic
                  ) -> o3d.geometry.PointCloud:
    rgb_o3d   = o3d.geometry.Image(
        cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB).astype(np.uint8))
    depth_o3d = o3d.geometry.Image((depth_m * 1000).astype(np.uint16))
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_o3d, depth_o3d, 1000.0, 3.5, False)
    return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intr)


# ─────────────────────────────────────────────────────────────────────────────
# TEASER++ + Colored ICP
# ─────────────────────────────────────────────────────────────────────────────

def run_teaser(A: np.ndarray, B: np.ndarray,
               noise_bound: float = 0.1) -> np.ndarray:
    """A, B: 3×N arrays.  Returns 4×4 T that maps A → B."""
    p = teaserpp_python.RobustRegistrationSolver.Params()
    p.cbar2           = 1.0
    p.noise_bound     = noise_bound
    p.estimate_scaling = True
    p.inlier_selection_mode = (
        teaserpp_python.RobustRegistrationSolver.INLIER_SELECTION_MODE.PMC_EXACT)
    p.rotation_tim_graph = (
        teaserpp_python.RobustRegistrationSolver.INLIER_GRAPH_FORMULATION.CHAIN)
    p.rotation_estimation_algorithm = (
        teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS)
    p.rotation_gnc_factor     = 1.4
    p.rotation_max_iterations = 10000
    p.rotation_cost_threshold = 1e-16
    s = teaserpp_python.RobustRegistrationSolver(p)
    s.solve(A, B)
    sol = s.getSolution()
    T = np.eye(4)
    T[:3, :3] = sol.rotation
    T[:3,  3] = sol.translation
    return T


def colored_icp(src: o3d.geometry.PointCloud,
                dst: o3d.geometry.PointCloud,
                T_init: np.ndarray,
                radius: float = 0.03) -> np.ndarray:
    vs = 0.02
    sd = src.voxel_down_sample(vs)
    dd = dst.voxel_down_sample(vs)
    kd = o3d.geometry.KDTreeSearchParamHybrid(vs * 2, max_nn=30)
    sd.estimate_normals(kd)
    dd.estimate_normals(kd)
    res = o3d.pipelines.registration.registration_colored_icp(
        sd, dd, radius, T_init,
        o3d.pipelines.registration.TransformationEstimationForColoredICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-9, relative_rmse=1e-9, max_iteration=1000))
    return res.transformation


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class DentMappingNode(Node):

    # ── init ──────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('dent_mapping_node')
        self._declare_params()
        self._read_params()
        self._init_models()
        self._init_state()
        self._init_publishers()

        if self.source == 'live':
            self._setup_live()
        else:
            self._setup_bag()

        self.get_logger().info(
            f'DentMappingNode ready  |  source={self.source}  '
            f'|  device={self.device}  '
            f'|  YOLO={self.yolo_model_path}')

    # ── parameters ────────────────────────────────────────────────────────────

    def _declare_params(self):
        S  = ParameterType.PARAMETER_STRING
        SA = ParameterType.PARAMETER_STRING_ARRAY
        D  = ParameterType.PARAMETER_DOUBLE
        I  = ParameterType.PARAMETER_INTEGER
        IA = ParameterType.PARAMETER_INTEGER_ARRAY

        def dp(n, v, t, desc=''):
            self.declare_parameter(n, v, ParameterDescriptor(type=t, description=desc))

        dp('source',          'live',   S,  '"live" or "bag"')
        dp('yolo_model_path', 'yolov8n.pt', S,
           'Path to YOLO .pt weights; auto-downloads if not found')
        dp('yolo_conf',       0.4,      D,  'YOLO confidence threshold')
        dp('yolo_class_ids',  [],       IA, 'Class ids to treat as dents ([] = all)')
        dp('dent_min_depth',  0.1,      D,  'Min valid depth for dent patch (m)')
        dp('dent_max_depth',  3.5,      D,  'Max valid depth for dent patch (m)')
        dp('voxel_size',      0.01,     D)
        dp('queue_size',      5,        I)
        dp('map_frame',       'map',    S)
        dp('camera_frame',    'camera_color_optical_frame', S)
        dp('fx', 593.698, D)
        dp('fy', 592.763, D)
        dp('cx', 318.613, D)
        dp('cy', 234.720, D)
        dp('bag_rgb_paths',   [], SA)
        dp('bag_depth_paths', [], SA)

    @staticmethod
    def _as_float(v) -> float:
        """
        LaunchConfiguration substitutions arrive as strings even when the
        parameter type is DOUBLE.  Coerce safely.
        """
        if isinstance(v, str):
            return float(v)
        return float(v)

    def _read_params(self):
        g = self.get_parameter
        self.source          = g('source').value
        self.yolo_model_path = g('yolo_model_path').value
        self.yolo_conf       = self._as_float(g('yolo_conf').value)
        self.yolo_class_ids  = set(g('yolo_class_ids').value)   # empty = accept all
        self.dent_min_depth  = self._as_float(g('dent_min_depth').value)
        self.dent_max_depth  = self._as_float(g('dent_max_depth').value)
        self.voxel_size      = self._as_float(g('voxel_size').value)
        self.map_frame       = g('map_frame').value
        self.cam_frame       = g('camera_frame').value
        self.fx = self._as_float(g('fx').value)
        self.fy = self._as_float(g('fy').value)
        self.cx = self._as_float(g('cx').value)
        self.cy = self._as_float(g('cy').value)
        self._intrinsics_ready = (self.source == 'bag')

    # ── model init ────────────────────────────────────────────────────────────

    def _init_models(self):
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        # YOLO ─────────────────────────────────────────────────────────────────
        self.get_logger().info(f'Loading YOLO from: {self.yolo_model_path}')
        self.yolo = YOLO(self.yolo_model_path)
        self.yolo.to(self.device)

        # SuperGlue ────────────────────────────────────────────────────────────
        sg_cfg = {
            'superpoint': {'nms_radius': 4, 'keypoint_threshold': 0.005,
                           'max_keypoints': 1024},
            'superglue':  {'weights': 'indoor', 'sinkhorn_iterations': 20,
                           'match_threshold': 0.2},
        }
        self.matching = Matching(sg_cfg).eval().to(self.device)

        # Open3D camera intrinsic (updated in live mode from /camera_info)
        self.o3d_intr = o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)

    # ── state ─────────────────────────────────────────────────────────────────

    def _init_state(self):
        self.lock           = Lock()
        self.frame_idx      = 0
        self.T_global       = np.eye(4)
        self.scene_combined = o3d.geometry.PointCloud()
        # Frozen dent patches – never moved after insertion
        self.dent_patches: List[DentPatch] = []
        self.prev_frame     = None   # (inp_sg, depth_np, pcd_o3d, stamp)

    # ── publishers ────────────────────────────────────────────────────────────

    def _init_publishers(self):
        qos = rclpy.qos.QoSProfile(depth=10)
        self.pub_scene     = self.create_publisher(PointCloud2,
            '/dent_mapping/scene_cloud',     qos)
        self.pub_dent      = self.create_publisher(PointCloud2,
            '/dent_mapping/dent_cloud',      qos)
        self.pub_detect    = self.create_publisher(Image,
            '/dent_mapping/detection_image', qos)
        self.pub_overlay   = self.create_publisher(Image,
            '/dent_mapping/overlay_image',   qos)
        self.tf_br         = tf2_ros.TransformBroadcaster(self)
        self.bridge        = CvBridge()

    # ── live mode ─────────────────────────────────────────────────────────────

    def _setup_live(self):
        from message_filters import ApproximateTimeSynchronizer, Subscriber
        self.get_logger().info('Live mode — subscribing to RealSense D415.')

        self._sub_rgb   = Subscriber(self, Image, '/camera/color/image_raw')
        self._sub_depth = Subscriber(self, Image, '/camera/depth/image_rect_raw')
        self._sub_info  = self.create_subscription(
            CameraInfo, '/camera/color/camera_info',
            self._camera_info_cb, 1)

        qs = self.get_parameter('queue_size').value
        self._sync = ApproximateTimeSynchronizer(
            [self._sub_rgb, self._sub_depth], queue_size=qs, slop=0.05)
        self._sync.registerCallback(self._rgbd_cb)

    def _camera_info_cb(self, msg: CameraInfo):
        if not self._intrinsics_ready:
            K = msg.k
            self.fx, self.cx = K[0], K[2]
            self.fy, self.cy = K[4], K[5]
            self.o3d_intr = o3d.camera.PinholeCameraIntrinsic(
                msg.width, msg.height, self.fx, self.fy, self.cx, self.cy)
            self._intrinsics_ready = True
            self.get_logger().info(
                f'Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
                f'cx={self.cx:.1f} cy={self.cy:.1f}')

    def _rgbd_cb(self, rgb_msg: Image, depth_msg: Image):
        if not self._intrinsics_ready:
            return
        rgb_np   = self.bridge.imgmsg_to_cv2(rgb_msg,   'bgr8')
        depth_np = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1').astype(np.float64) / 1000.0
        with self.lock:
            self._process_frame(rgb_np, depth_np, rgb_msg.header.stamp)

    # ── bag / offline mode ────────────────────────────────────────────────────

    def _setup_bag(self):
        self._bag_rgb   = self.get_parameter('bag_rgb_paths').value
        self._bag_depth = self.get_parameter('bag_depth_paths').value
        self._bag_idx   = 0
        if not self._bag_rgb:
            self.get_logger().error('bag_rgb_paths is empty.')
            return
        self._bag_timer = self.create_timer(1.0, self._bag_tick)

    def _bag_tick(self):
        if self._bag_idx >= len(self._bag_rgb):
            self.get_logger().info('Bag playback complete.')
            self._bag_timer.cancel()
            return
        rgb_np   = cv2.imread(self._bag_rgb[self._bag_idx])
        depth_np = cv2.imread(self._bag_depth[self._bag_idx],
                              cv2.IMREAD_UNCHANGED).astype(np.float64)
        if depth_np.max() > 20:
            depth_np /= 1000.0
        self._bag_idx += 1
        stamp = self.get_clock().now().to_msg()
        with self.lock:
            self._process_frame(rgb_np, depth_np, stamp)

    # ─────────────────────────────────────────────────────────────────────────
    # Core per-frame processing
    # ─────────────────────────────────────────────────────────────────────────

    def _process_frame(self, rgb_np: np.ndarray, depth_np: np.ndarray, stamp):
        idx = self.frame_idx
        self.frame_idx += 1

        # ── 1. YOLO detection ─────────────────────────────────────────────────
        detections = self._run_yolo(rgb_np)   # list of (x1,y1,x2,y2,conf,cls_id)
        detect_vis = self._draw_detections(rgb_np.copy(), detections)

        # ── 2. Freeze any detected dent ROIs into map frame ───────────────────
        #    We freeze NOW, using the current T_global, so the patch stays
        #    exactly where the camera was when it saw the dent.
        for det in detections:
            patch_pcd = self._backproject_bbox_roi(det, depth_np)
            if patch_pcd is None:
                continue
            # Transform: camera → map  (T_global maps camera_0 → camera_N,
            # so we apply FLIP then T_global to land in map frame)
            patch_global = copy.deepcopy(patch_pcd)
            patch_global.transform(FLIP)
            patch_global.transform(self.T_global)
            patch_global.paint_uniform_color(DENT_COLOUR)

            self.dent_patches.append(DentPatch(
                frame_idx  = idx,
                confidence = float(det[4]),
                class_id   = int(det[5]),
                bbox_xyxy  = (int(det[0]), int(det[1]),
                               int(det[2]), int(det[3])),
                pcd        = patch_global,
            ))
            self.get_logger().info(
                f'[YOLO] Frame {idx}: dent frozen  '
                f'conf={det[4]:.2f}  cls={int(det[5])}  '
                f'pts={len(patch_global.points)}')

        # ── 3. Build Open3D RGBD point cloud for this frame ───────────────────
        pcd_cur = make_rgbd_pcd(rgb_np, depth_np, self.o3d_intr)

        # ── 4. SuperGlue feature tensor for registration ──────────────────────
        tmp = Path(f'/tmp/_dm_{idx}.png')
        cv2.imwrite(str(tmp), rgb_np)
        _, inp_cur, _ = read_image(tmp, self.device, [640, 480], 0, False)

        if self.prev_frame is None:
            pcd_cur_g = copy.deepcopy(pcd_cur)
            pcd_cur_g.transform(FLIP)
            self.scene_combined += pcd_cur_g
            self.prev_frame = (inp_cur, depth_np, pcd_cur, stamp)
            self._publish_all(stamp, detect_vis, rgb_np, detections)
            return

        # ── 5. SuperGlue matching + TEASER + Colored ICP ──────────────────────
        inp_prev, depth_prev, _, stamp_prev = self.prev_frame

        with torch.no_grad():
            pred = self.matching({'image0': inp_prev, 'image1': inp_cur})
        pred = {k: v[0].detach().cpu().numpy() for k, v in pred.items()}

        matches = pred['matches0']
        valid   = matches > -1
        mkpts0  = pred['keypoints0'][valid]
        mkpts1  = pred['keypoints1'][matches[valid]]

        pts0, vmask0 = backproject_kpts(mkpts0, depth_prev, self.fx, self.fy,
                                        self.cx, self.cy)
        pts1, vmask1 = backproject_kpts(mkpts1, depth_np,   self.fx, self.fy,
                                        self.cx, self.cy)

        # Keep only mutually valid
        joint_valid = vmask0 & vmask1[:len(vmask0)] if len(vmask0) <= len(vmask1) \
                      else vmask0[:len(vmask1)] & vmask1
        n = min(joint_valid.sum(), len(pts0), len(pts1))

        if n < 10:
            self.get_logger().warn(
                f'Frame {idx}: only {n} 3-D correspondences — skipping reg.')
            self.prev_frame = (inp_cur, depth_np, pcd_cur, stamp)
            self._publish_all(stamp, detect_vis, rgb_np, detections)
            return

        A = pts0[:n].T
        B = pts1[:n].T

        T_teaser = run_teaser(A, B, noise_bound=self.voxel_size)

        pcd_pts0 = o3d.geometry.PointCloud()
        pcd_pts0.points = o3d.utility.Vector3dVector(pts0[:n])
        pcd_pts1 = o3d.geometry.PointCloud()
        pcd_pts1.points = o3d.utility.Vector3dVector(pts1[:n])
        T_icp = colored_icp(pcd_pts0, pcd_pts1, T_teaser)

        # ── 6. Update T_global and accumulate scene cloud ─────────────────────
        if idx == 1:
            pcd_cur_g = copy.deepcopy(pcd_cur)
            pcd_cur_g.transform(T_icp)
            pcd_cur_g.transform(FLIP)
            self.T_global = T_icp.copy()
        else:
            self.T_global = self.T_global @ np.linalg.inv(T_icp)
            pcd_cur_g = copy.deepcopy(pcd_cur)
            pcd_cur_g.transform(self.T_global)
            pcd_cur_g.transform(FLIP)

        self.scene_combined += pcd_cur_g

        # ── 7. TF broadcasts ──────────────────────────────────────────────────
        self.tf_br.sendTransform(
            mat_to_ts(T_icp, f'frame_{idx-1}', f'frame_{idx}', stamp))
        self.tf_br.sendTransform(
            mat_to_ts(self.T_global, self.map_frame, self.cam_frame, stamp))

        # ── 8. Publish ────────────────────────────────────────────────────────
        self._publish_all(stamp, detect_vis, rgb_np, detections)
        self.prev_frame = (inp_cur, depth_np, pcd_cur, stamp)

    # ─────────────────────────────────────────────────────────────────────────
    # YOLO helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _run_yolo(self, rgb_bgr: np.ndarray) -> list:
        """
        Run YOLO on the BGR frame.
        Returns list of (x1, y1, x2, y2, conf, class_id).
        """
        results = self.yolo.predict(
            source     = rgb_bgr,
            conf       = self.yolo_conf,
            verbose    = False,
            device     = self.device,
        )
        detections = []
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                cls_id = int(box.cls[0].item())
                # Filter by class if yolo_class_ids is configured
                if self.yolo_class_ids and cls_id not in self.yolo_class_ids:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].item())
                detections.append((x1, y1, x2, y2, conf, cls_id))
        return detections

    def _draw_detections(self, img: np.ndarray, detections: list) -> np.ndarray:
        """Draw YOLO bounding boxes on a copy of the image."""
        for (x1, y1, x2, y2, conf, cls_id) in detections:
            cv2.rectangle(img,
                          (int(x1), int(y1)), (int(x2), int(y2)),
                          (0, 60, 220), 2)
            label = f'dent {conf:.2f}'
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX,
                                          0.55, 1)
            cv2.rectangle(img,
                          (int(x1), int(y1) - th - 6),
                          (int(x1) + tw + 4, int(y1)),
                          (0, 60, 220), -1)
            cv2.putText(img, label,
                        (int(x1) + 2, int(y1) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                        cv2.LINE_AA)
        return img

    def _backproject_bbox_roi(self, det: tuple,
                               depth_np: np.ndarray
                               ) -> Optional[o3d.geometry.PointCloud]:
        """
        Dense-backproject the pixels inside a YOLO bbox ROI using the depth
        frame, returning an Open3D PointCloud in camera frame.
        Returns None if too few valid points.
        """
        x1, y1, x2, y2, conf, cls_id = det
        x1, y1 = max(0, int(x1)), max(0, int(y1))
        x2 = min(depth_np.shape[1] - 1, int(x2))
        y2 = min(depth_np.shape[0] - 1, int(y2))

        mask2d = np.zeros(depth_np.shape[:2], dtype=bool)
        mask2d[y1:y2+1, x1:x2+1] = True

        pts3d = backproject_mask(
            mask2d, depth_np, self.fx, self.fy, self.cx, self.cy,
            d_min=self.dent_min_depth, d_max=self.dent_max_depth)

        if len(pts3d) < 10:
            self.get_logger().warn(
                f'Dent bbox ({x1},{y1})→({x2},{y2}) has only '
                f'{len(pts3d)} valid depth pts — skipping.')
            return None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts3d)
        return pcd

    # ─────────────────────────────────────────────────────────────────────────
    # Publish helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _build_dent_cloud(self) -> o3d.geometry.PointCloud:
        """Merge all frozen dent patches into one PointCloud."""
        combined = o3d.geometry.PointCloud()
        for patch in self.dent_patches:
            combined += patch.pcd
        return combined

    def _build_overlay(self, rgb_np: np.ndarray,
                        detections: list) -> np.ndarray:
        """
        Semi-transparent coloured fill over each detected bbox.
        """
        overlay = rgb_np.copy()
        for (x1, y1, x2, y2, conf, cls_id) in detections:
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            roi = overlay[y1:y2, x1:x2]
            tint = np.full_like(roi, (30, 30, 200))   # BGR red tint
            cv2.addWeighted(tint, 0.40, roi, 0.60, 0, roi)
            overlay[y1:y2, x1:x2] = roi
        return overlay

    def _publish_all(self, stamp, detect_vis: np.ndarray,
                     rgb_np: np.ndarray, detections: list):
        vs = self.voxel_size

        # Scene cloud
        scene_ds = self.scene_combined.voxel_down_sample(vs)
        self.pub_scene.publish(o3d_to_pc2(scene_ds, self.map_frame, stamp))

        # Dent cloud  (all frozen patches merged)
        dent_merged = self._build_dent_cloud()
        if len(dent_merged.points) > 0:
            dent_ds = dent_merged.voxel_down_sample(vs)
        else:
            dent_ds = dent_merged
        self.pub_dent.publish(o3d_to_pc2(dent_ds, self.map_frame, stamp))

        # Detection image (YOLO bbox visualisation)
        det_msg = self.bridge.cv2_to_imgmsg(detect_vis, 'bgr8')
        det_msg.header.stamp    = stamp
        det_msg.header.frame_id = self.cam_frame
        self.pub_detect.publish(det_msg)

        # Overlay image (tinted ROI fill)
        if detections:
            ov = self._build_overlay(rgb_np, detections)
            ov_msg = self.bridge.cv2_to_imgmsg(ov, 'bgr8')
            ov_msg.header.stamp    = stamp
            ov_msg.header.frame_id = self.cam_frame
            self.pub_overlay.publish(ov_msg)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = DentMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
