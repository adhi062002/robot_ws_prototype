#!/usr/bin/env python3
"""
dent_reconstruction_node.py

Stage 2 of the dent-mapping pipeline (YOLO -> RECONSTRUCTION -> POLISHING).

Wraps the working dent_reconstruction_v2.py algorithm (SuperPoint+SuperGlue
matching, TEASER++ global registration, colored ICP refinement, dent-crop
overlay compositing) as a ROS 2 action server, so the GUI can fire it as a
discrete pipeline stage after the YOLO node finishes.

Input contract
--------------
input_dir is supplied explicitly in the action goal -- normally the YOLO
node's output_dir from the previous stage's result, NOT auto-discovered
from ~/datasets. It must contain:
    <input_dir>/rgb/*.png          -- post-YOLO-detection frames
    <input_dir>/depth/*.png        -- unchanged, passed through by YOLO
    <input_dir>/intrinsics.json    -- unchanged, passed through by YOLO

dent_dir defaults to <input_dir>/dent (where the YOLO node writes detected
dent crops). Each detection is a matched pair of files:
    <dent_dir>/<stem>_NNN.png    -- pixel crop of the bbox region
    <dent_dir>/<stem>_NNN.json   -- {"frame": "<stem>.png", "bbox": [x1,y1,x2,y2]}
where bbox is in pixel coordinates of the FULL source frame (not the
crop's own coordinates). Because YOLO records exactly which frame each
crop came from, this node no longer needs to search for or estimate
where a dent goes -- it looks the frame up directly by filename and
pastes the crop back at its known bbox. (This replaces the original
script's SuperGlue-search + affine-warp approach for dent placement,
which was needed only because the source frame wasn't known in advance.)

output_dir defaults to input_dir itself (reconstruction.ply is written
alongside rgb/depth/intrinsics.json, same convention as the original
script's default).

3D dent marking: each dent's pixel bbox is backprojected into 3D using
that frame's own depth image and the loaded camera intrinsics, then
carried through the exact same transform (T_global + FLIP) used to place
that frame's data into the merged scene. The result is a bright red point
outline tracing the dent's bbox, added to the final point cloud AFTER
downsampling -- so it isn't diluted or voxel-merged away, and shows up
clearly in any .ply viewer (CloudCompare, MeshLab, Open3D, etc.) at the
dent's actual 3D location.

Threading / execution model
----------------------------
This is a synchronous, CPU/GPU-heavy pipeline (SuperGlue on GPU, TEASER++,
ICP) -- NOT event-driven like the capture node. execute_callback is a
plain blocking function (rclpy's action executor does not run a real
asyncio loop, so this must not be `async def` -- see realsense_frame_
capture_node.py for the "no running event loop" pitfall this avoids).
It runs on the node's ReentrantCallbackGroup thread pool, so it won't
block the action server from accepting/rejecting new goals, but only one
reconstruction goal is allowed to run at a time (enforced in goal_callback,
same pattern as the capture node) since the SuperGlue model instance and
GPU are not meant to be shared across concurrent reconstructions.

The SuperPoint+SuperGlue model is loaded ONCE in __init__, not per-goal,
since model load is expensive and the node is expected to run repeatedly
across many capture runs.

Cancellation is checked between dent-paste iterations (pass 1) and
between frame-pair registrations (pass 2) -- the two loops that scale
with capture length. Feedback (stage name + progress fraction) is
published at the same checkpoints.

Visualization: o3d.visualization.draw_geometries() is a GUI-blocking call
that cannot run inside a ROS action callback, so this node always runs
headless (no VISUALIZE branches). Since dent placement is now a direct
bbox paste (see input contract below) rather than an interactive mask-
drawing step, there's no longer a manual/GUI dependency in this node at
all -- it can run fully unattended as long as YOLO's output matches the
contract below.
"""

import json
import time
from pathlib import Path
from threading import Lock

import cv2
import numpy as np
import open3d as o3d
import teaserpp_python
import torch

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from models.matching import Matching
from models.utils import read_image

from dent_mapping_interfaces.action import ReconstructScan


class GoalCancelled(Exception):
    """Raised internally to unwind out of the pipeline on cancellation."""


# ---------------------------------------------------------------------------
# Pure helper functions -- unchanged logic from dent_reconstruction_v2.py.
# ---------------------------------------------------------------------------

def backproject(kpts, depth, fx, fy, cx, cy):
    """Returns (pts3d, valid_mask) instead of pre-filtering, so callers can
    combine masks across two point sets before indexing either one."""
    z = depth[np.clip(kpts[:, 1].astype(int), 0, depth.shape[0] - 1),
              np.clip(kpts[:, 0].astype(int), 0, depth.shape[1] - 1)]
    x = (kpts[:, 0] - cx) * z / fx
    y = (kpts[:, 1] - cy) * z / fy
    pts3d = np.vstack((x, y, z)).T
    valid = (z > 0) & np.isfinite(z)
    return pts3d, valid


def load_intrinsics(intrinsics_path, default=(593.698, 592.763, 318.613, 234.720)):
    if intrinsics_path and Path(intrinsics_path).exists():
        with open(intrinsics_path) as f:
            data = json.load(f)
        return data["fx"], data["fy"], data["cx"], data["cy"]
    return default


def discover_frames(scan_dir):
    """Glob <scan_dir>/rgb and <scan_dir>/depth, sorted by filename, and
    verify they line up 1:1 (same count, same stems) before returning."""
    scan_dir = Path(scan_dir)
    rgb_dir, depth_dir = scan_dir / "rgb", scan_dir / "depth"

    rgb_files = sorted(rgb_dir.glob("*.png"))
    depth_files = sorted(depth_dir.glob("*.png"))

    if not rgb_files:
        raise FileNotFoundError(f"No rgb frames found in {rgb_dir}")
    if len(rgb_files) != len(depth_files):
        raise ValueError(
            f"rgb/depth count mismatch: {len(rgb_files)} rgb vs {len(depth_files)} depth "
            f"in {scan_dir} -- capture may have been interrupted mid-frame."
        )
    for r, d in zip(rgb_files, depth_files):
        if r.stem != d.stem:
            raise ValueError(f"rgb/depth filename mismatch: {r.name} vs {d.name}")

    return [str(p) for p in rgb_files], [str(p) for p in depth_files]


def imread_checked(path):
    img = cv2.imread(str(path))
    if img is None:
        raise FileNotFoundError(f"cv2.imread failed to load: {path}")
    return img


def depth_read_checked(path):
    d = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if d is None:
        raise FileNotFoundError(f"cv2.imread failed to load depth: {path}")
    return d.astype(np.float64)


def sample_bbox_perimeter(bbox, step=6):
    """Return pixel coords tracing the rectangle's perimeter, spaced ~step
    pixels apart, so backprojecting them gives a visible 3D outline rather
    than just 4 isolated corner points."""
    x1, y1, x2, y2 = bbox
    pts = []
    for x in range(x1, x2 + 1, step):
        pts.append((x, y1))
        pts.append((x, y2))
    for y in range(y1, y2 + 1, step):
        pts.append((x1, y))
        pts.append((x2, y))
    return np.array(pts, dtype=np.float64) if pts else np.zeros((0, 2))


VOXEL_SIZE = 0.1
DEPTH_TRUNC = 3.5


class DentReconstructionNode(Node):
    def __init__(self):
        super().__init__("dent_reconstruction_node")

        self.declare_parameter("dent_subdir", "dent")

        self.action_lock = Lock()
        self.active_goal_handle = None
        self.io_callback_group = ReentrantCallbackGroup()

        self.get_logger().info("Loading SuperPoint+SuperGlue model...")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        config = {
            "superpoint": {"nms_radius": 4, "keypoint_threshold": 0.005, "max_keypoints": 1024},
            "superglue": {"weights": "indoor", "sinkhorn_iterations": 20, "match_threshold": 0.2},
        }
        self.matching = Matching(config).eval().to(self.device)
        self.get_logger().info(f"Model loaded on device={self.device}.")

        self.action_server = ActionServer(
            self,
            ReconstructScan,
            "reconstruct_scan",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.io_callback_group,
        )

        self.get_logger().info(
            "Dent reconstruction action server ready. Action: /reconstruct_scan"
        )

    # ------------------------------------------------------------------
    # ROS 2 Action handling
    # ------------------------------------------------------------------

    def goal_callback(self, goal_request):
        with self.action_lock:
            if self.active_goal_handle is not None:
                self.get_logger().warn(
                    "A reconstruction is already running; rejecting new goal."
                )
                return GoalResponse.REJECT

            if not goal_request.input_dir:
                self.get_logger().error("Goal rejected: input_dir is empty.")
                return GoalResponse.REJECT

            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Reconstruction cancellation requested.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        with self.action_lock:
            self.active_goal_handle = goal_handle

        req = goal_handle.request
        input_dir = Path(req.input_dir).expanduser()
        dent_dir = (
            Path(req.dent_dir).expanduser()
            if req.dent_dir
            else input_dir / self.get_parameter("dent_subdir").value
        )
        output_dir = Path(req.output_dir).expanduser() if req.output_dir else input_dir
        ply_path = output_dir / "reconstruction.ply"

        try:
            scene = self._run_pipeline(goal_handle, input_dir, dent_dir, output_dir, ply_path)
            goal_handle.succeed()
            return self._make_result(True, output_dir, ply_path, "")

        except GoalCancelled:
            goal_handle.canceled()
            return self._make_result(False, output_dir, ply_path, "Cancelled by request.")

        except Exception as exc:
            self.get_logger().error(f"Reconstruction failed: {exc}")
            goal_handle.abort()
            return self._make_result(False, output_dir, ply_path, str(exc))

        finally:
            with self.action_lock:
                self.active_goal_handle = None

    def _make_result(self, success, output_dir, ply_path, message):
        result = ReconstructScan.Result()
        result.success = bool(success)
        result.output_dir = str(output_dir)
        result.ply_path = str(ply_path) if success else ""
        result.message = message
        return result

    def _publish_feedback(self, goal_handle, stage, progress):
        feedback = ReconstructScan.Feedback()
        feedback.stage = stage
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)

    def _check_cancel(self, goal_handle):
        if goal_handle.is_cancel_requested:
            raise GoalCancelled()

    # ------------------------------------------------------------------
    # Pipeline (adapted from dent_reconstruction_v2.py main()).
    # Algorithm/math unchanged from the working script -- only feedback,
    # cancellation checks, and headless enforcement were added.
    # ------------------------------------------------------------------

    def _run_pipeline(self, goal_handle, input_dir, dent_dir, output_dir, ply_path):
        start = time.time()

        if not input_dir.exists():
            raise FileNotFoundError(f"Input directory does not exist: {input_dir}")

        output_dir.mkdir(parents=True, exist_ok=True)

        self._publish_feedback(goal_handle, "discovering_frames", 0.0)
        rgb_paths, depth_paths = discover_frames(input_dir)
        fx, fy, cx, cy = load_intrinsics(input_dir / "intrinsics.json")
        self.get_logger().info(f"Discovered {len(rgb_paths)} frame pairs in {input_dir}")

        rgb_paths_orig = list(rgb_paths)
        rgb_paths_recon = list(rgb_paths)

        dent_list = sorted(dent_dir.glob("*.json")) if dent_dir.exists() else []
        if not dent_list:
            self.get_logger().warn(
                f"No dent records found in {dent_dir}; running plain reconstruction "
                "with no dent overlay."
            )

        dent_hosts = {}
        dent_bboxes = {}  # frame_idx -> list of clamped (x1, y1, x2, y2)

        # Map frame stem -> index into rgb_paths_orig, so we can look a dent
        # crop's source frame up directly instead of searching for it.
        stem_to_idx = {Path(p).stem: idx for idx, p in enumerate(rgb_paths_orig)}

        # ---------------- PASS 1: paste dent crops onto their known source frame ----------------
        # No SuperGlue search needed here anymore: the YOLO node crops each
        # detection directly out of its source frame and records that
        # frame's filename + the crop's pixel bbox in a sibling .json, so
        # placement is an exact ROI paste, not an estimated affine warp.
        n_dents = len(dent_list)
        for dent_idx, dent_json_path in enumerate(dent_list):
            self._check_cancel(goal_handle)
            self._publish_feedback(
                goal_handle, f"pasting_dent_{dent_idx + 1}_of_{n_dents}",
                0.5 * (dent_idx / max(n_dents, 1)),
            )

            with open(dent_json_path) as f:
                det = json.load(f)

            frame_name = det.get("frame")
            bbox = det.get("bbox")
            if not frame_name or not bbox or len(bbox) != 4:
                self.get_logger().warn(
                    f"Malformed dent record {dent_json_path.name} "
                    f"(need 'frame' and 4-element 'bbox'), skipping."
                )
                continue

            frame_stem = Path(frame_name).stem
            if frame_stem not in stem_to_idx:
                self.get_logger().warn(
                    f"{dent_json_path.name} references frame '{frame_name}', "
                    f"which was not found in {input_dir / 'rgb'} -- skipping."
                )
                continue
            frame_idx = stem_to_idx[frame_stem]

            crop_path = dent_json_path.with_suffix(".png")
            if not crop_path.exists():
                self.get_logger().warn(
                    f"Expected crop image {crop_path.name} next to "
                    f"{dent_json_path.name} but it's missing -- skipping."
                )
                continue

            self.get_logger().info(
                f"Processing dent {dent_json_path.stem}: frame idx {frame_idx} "
                f"({frame_name}), bbox={bbox}"
            )

            crop_img = imread_checked(crop_path)
            base_img_path = rgb_paths_recon[frame_idx]
            rgb_color = imread_checked(base_img_path)
            h, w = rgb_color.shape[:2]

            x1, y1, x2, y2 = [int(round(v)) for v in bbox]
            x1, x2 = sorted((max(0, min(x1, w)), max(0, min(x2, w))))
            y1, y2 = sorted((max(0, min(y1, h)), max(0, min(y2, h))))
            box_w, box_h = x2 - x1, y2 - y1
            if box_w <= 0 or box_h <= 0:
                self.get_logger().warn(
                    f"Degenerate bbox {bbox} for {dent_json_path.name}, skipping."
                )
                continue

            # Resize defensively in case the crop's saved size doesn't
            # exactly match the bbox (e.g. bbox was rounded differently
            # upstream) -- keeps this robust without needing an exact
            # pixel-for-pixel guarantee from the YOLO node.
            if crop_img.shape[:2] != (box_h, box_w):
                crop_img = cv2.resize(crop_img, (box_w, box_h))

            overlay = rgb_color.copy()
            roi = overlay[y1:y2, x1:x2]
            overlay[y1:y2, x1:x2] = cv2.addWeighted(roi, 1, crop_img, 0.5, 0)

            host_stem = Path(rgb_paths_orig[frame_idx]).stem
            out_overlay = Path(rgb_paths_orig[frame_idx]).parent / f"{host_stem}_overlay.png"
            if not cv2.imwrite(str(out_overlay), overlay):
                raise RuntimeError(f"Failed to write dent overlay: {out_overlay}")

            rgb_paths_recon[frame_idx] = str(out_overlay)
            dent_hosts.setdefault(frame_idx, []).append(dent_json_path.stem)
            dent_bboxes.setdefault(frame_idx, []).append((x1, y1, x2, y2))

        dent_frame_indices = set(dent_hosts.keys())
        self.get_logger().info(f"Dent host frame indices: {dent_frame_indices}")

        # ---------------- PASS 2: pairwise registration + fusion ----------------
        scene_combined = o3d.geometry.PointCloud()
        dent_pcd_map = {}
        dent_transforms = {}  # frame_idx -> 4x4 transform placing that frame into scene_down
        T_global = np.eye(4)

        n_pairs = max(len(rgb_paths_recon) - 1, 1)
        for i in range(len(rgb_paths_recon) - 1):
            self._check_cancel(goal_handle)
            self._publish_feedback(
                goal_handle, f"registering_pair_{i + 1}_of_{n_pairs}",
                0.5 + 0.5 * (i / n_pairs),
            )

            rgb0 = Path(rgb_paths_recon[i])
            rgb1 = Path(rgb_paths_recon[i + 1])
            depth0 = Path(depth_paths[i])
            depth1 = Path(depth_paths[i + 1])

            image0, inp0, scales0 = read_image(rgb0, self.device, [640, 480], 0, False)
            image1, inp1, scales1 = read_image(rgb1, self.device, [640, 480], 0, False)

            d0_raw = depth_read_checked(depth0)
            d1_raw = depth_read_checked(depth1)

            depth_scale = 1000.0 if d0_raw.max() > 20 else 1.0
            d0, d1 = d0_raw / depth_scale, d1_raw / depth_scale

            with torch.no_grad():
                pred = self.matching({"image0": inp0, "image1": inp1})
            pred = {k: v[0].detach().cpu().numpy() for k, v in pred.items()}

            kpts0, kpts1 = pred["keypoints0"], pred["keypoints1"]
            matches, conf = pred["matches0"], pred["matching_scores0"]
            valid = matches > -1
            mkpts0 = kpts0[valid]
            mkpts1 = kpts1[matches[valid]]

            pts3d_0_all, valid0 = backproject(mkpts0, d0, fx, fy, cx, cy)
            pts3d_1_all, valid1 = backproject(mkpts1, d1, fx, fy, cx, cy)
            valid = valid0 & valid1
            pts3d_0 = pts3d_0_all[valid]
            pts3d_1 = pts3d_1_all[valid]

            if pts3d_0.shape[0] < 3:
                self.get_logger().warn(
                    f"Only {pts3d_0.shape[0]} valid 3D correspondences for pair "
                    f"{i}/{i + 1} (need >=3) -- skipping."
                )
                continue

            A_corr = np.ascontiguousarray(pts3d_0.T)
            B_corr = np.ascontiguousarray(pts3d_1.T)

            solver_params = teaserpp_python.RobustRegistrationSolver.Params()
            solver_params.cbar2 = 1.0
            solver_params.noise_bound = VOXEL_SIZE
            solver_params.estimate_scaling = True
            solver_params.inlier_selection_mode = (
                teaserpp_python.RobustRegistrationSolver.INLIER_SELECTION_MODE.PMC_EXACT
            )
            solver_params.rotation_tim_graph = (
                teaserpp_python.RobustRegistrationSolver.INLIER_GRAPH_FORMULATION.CHAIN
            )
            solver_params.rotation_estimation_algorithm = (
                teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
            )
            solver_params.rotation_gnc_factor = 1.4
            solver_params.rotation_max_iterations = 10000
            solver_params.rotation_cost_threshold = 1e-16
            teaser_solver = teaserpp_python.RobustRegistrationSolver(solver_params)
            teaser_solver.solve(A_corr, B_corr)
            solution = teaser_solver.getSolution()
            T_teaser = np.identity(4)
            T_teaser[:3, :3] = solution.rotation
            T_teaser[:3, 3] = solution.translation

            color0_o3d = o3d.io.read_image(str(rgb0))
            depth0_o3d = o3d.io.read_image(str(depth0))
            color1_o3d = o3d.io.read_image(str(rgb1))
            depth1_o3d = o3d.io.read_image(str(depth1))

            rgbd0 = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color0_o3d, depth0_o3d, depth_scale, DEPTH_TRUNC, False
            )
            rgbd1 = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color1_o3d, depth1_o3d, depth_scale, DEPTH_TRUNC, False
            )

            intrinsic = o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
            )
            compcd0 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd0, intrinsic)
            compcd1 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd1, intrinsic)

            compcd0_down = compcd0.voxel_down_sample(0.02)
            compcd1_down = compcd1.voxel_down_sample(0.02)
            compcd0_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.04, max_nn=30))
            compcd1_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.04, max_nn=30))

            result_icp = o3d.pipelines.registration.registration_colored_icp(
                compcd0_down, compcd1_down, 0.03, T_teaser,
                o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    relative_fitness=1e-9, relative_rmse=1e-9, max_iteration=1000
                ),
            )

            FLIP = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]

            if i == 0:
                compcd0.transform(result_icp.transformation)
                compcd0.transform(FLIP)
                compcd1.transform(FLIP)
                scene_combined += compcd0

                if (i + 1) in dent_frame_indices:
                    dent_pcd_map[i + 1] = compcd1
                    dent_transforms[i + 1] = np.array(FLIP) @ T_global
                else:
                    scene_combined += compcd1
            else:
                T_global = T_global @ np.linalg.inv(result_icp.transformation)
                compcd1.transform(T_global)
                compcd0.transform(FLIP)
                compcd1.transform(FLIP)

                if (i + 1) in dent_frame_indices:
                    dent_pcd_map[i + 1] = compcd1
                    dent_transforms[i + 1] = np.array(FLIP) @ T_global
                else:
                    scene_combined += compcd1

        self._check_cancel(goal_handle)
        self._publish_feedback(goal_handle, "downsampling_and_merging", 0.97)

        scene_down = scene_combined.voxel_down_sample(0.01)
        for frame_idx, pc in dent_pcd_map.items():
            scene_down += pc
            self.get_logger().info(
                f"Merged dent pointcloud from frame idx {frame_idx} "
                f"(dents: {dent_hosts.get(frame_idx)})"
            )

        self._check_cancel(goal_handle)
        self._publish_feedback(goal_handle, "marking_dents_in_3d", 0.98)

        DENT_MARKER_COLOR = [1.0, 0.0, 0.0]  # bright red, stands out in any viewer
        for frame_idx in dent_frame_indices:
            transform = dent_transforms.get(frame_idx)
            if transform is None:
                continue  # shouldn't happen, but don't let a marker crash the whole save

            depth_path = depth_paths[frame_idx]
            d_raw = depth_read_checked(depth_path)
            depth_scale = 1000.0 if d_raw.max() > 20 else 1.0
            d = d_raw / depth_scale

            for bbox in dent_bboxes.get(frame_idx, []):
                perim_px = sample_bbox_perimeter(bbox)
                if perim_px.shape[0] == 0:
                    continue

                pts3d, valid = backproject(perim_px, d, fx, fy, cx, cy)
                pts3d = pts3d[valid]
                if pts3d.shape[0] == 0:
                    self.get_logger().warn(
                        f"No valid depth along bbox perimeter for frame idx "
                        f"{frame_idx}, bbox {bbox} -- skipping 3D marker for this dent."
                    )
                    continue

                marker = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts3d))
                marker.paint_uniform_color(DENT_MARKER_COLOR)
                marker.transform(transform)
                scene_down += marker  # added after downsampling so it isn't diluted/merged away

        self._publish_feedback(goal_handle, "saving", 0.99)
        if not o3d.io.write_point_cloud(str(ply_path), scene_down):
            raise RuntimeError(f"Failed to write point cloud: {ply_path}")
        self.get_logger().info(
            f"Saved reconstruction to {ply_path} "
            f"(took {time.time() - start:.1f}s)"
        )
        self._publish_feedback(goal_handle, "done", 1.0)
        return scene_down


def main(args=None):
    rclpy.init(args=args)
    node = DentReconstructionNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
