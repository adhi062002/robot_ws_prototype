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

from dent_mapping import recon_core as rc
from dent_mapping_interfaces.action import ReconstructScan


class GoalCancelled(Exception):
    """Raised internally to unwind out of the pipeline on cancellation."""


# ---------------------------------------------------------------------------
# Pure helper functions -- unchanged logic from dent_reconstruction_v2.py.
# ---------------------------------------------------------------------------

def discover_frames(scan_dir):
    """Glob <scan_dir>/rgb and <scan_dir>/depth, sorted by filename, and
    verify they line up 1:1 (same count, same stems) before returning."""
    scan_dir = Path(scan_dir)
    rgb_dir, depth_dir = scan_dir / "rgb", scan_dir / "depth"

    # Older versions wrote <stem>_overlay.png into rgb/; ignore any left over.
    rgb_files = sorted(p for p in rgb_dir.glob("*.png") if not p.stem.endswith("_overlay"))
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


class DentReconstructionNode(Node):
    def __init__(self):
        super().__init__("dent_reconstruction_node")

        self.declare_parameter("dent_subdir", "dent")
        self.declare_parameter("dent_overlay_gain", 0.5)   # tint strength of dent crops, 0 = off
        # Fusion / geometry
        self.declare_parameter("voxel_size", 0.01)         # m, final cloud
        self.declare_parameter("depth_trunc", 3.5)         # m
        self.declare_parameter("marker_step_px", 6)
        # Feature matching (SuperGlue works at this size; keypoints are scaled back)
        self.declare_parameter("superglue_resize", [640, 480])
        # Registration. noise_bound must be about the 3D error of a good match:
        # D415 depth noise + SuperGlue pixel error at 0.3-1 m is a few mm, so 2 cm.
        self.declare_parameter("teaser_noise_bound", 0.02)
        self.declare_parameter("min_correspondences", 6)
        self.declare_parameter("icp_voxel", 0.02)
        self.declare_parameter("icp_max_corr", 0.03)
        # Pair acceptance gates -- a pair that fails them is REPORTED and splits the scene
        self.declare_parameter("min_icp_fitness", 0.30)
        self.declare_parameter("max_icp_rmse", 0.02)
        self.declare_parameter("max_correction_m", 0.05)
        self.declare_parameter("max_correction_deg", 10.0)

        self.action_lock = Lock()
        self.active_goal_handle = None
        self._last_warnings = []
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
            self._last_warnings = []
            self._run_pipeline(goal_handle, input_dir, dent_dir, output_dir, ply_path)
            goal_handle.succeed()
            return self._make_result(True, output_dir, ply_path, " | ".join(self._last_warnings))

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

    # ------------------------------------------------------------------
    # Registration helpers
    # ------------------------------------------------------------------

    def _reg_config(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        return rc.RegConfig(
            icp_voxel=float(g("icp_voxel")),
            icp_max_corr=float(g("icp_max_corr")),
            min_correspondences=int(g("min_correspondences")),
            min_icp_fitness=float(g("min_icp_fitness")),
            max_icp_rmse=float(g("max_icp_rmse")),
            max_correction_m=float(g("max_correction_m")),
            max_correction_deg=float(g("max_correction_deg")),
        )

    def _match_frames(self, rgb0, rgb1):
        """SuperPoint+SuperGlue matches as FULL-RESOLUTION pixel coordinates."""
        resize = [int(v) for v in self.get_parameter("superglue_resize").value]
        _, inp0, scales0 = read_image(Path(rgb0), self.device, resize, 0, False)
        _, inp1, scales1 = read_image(Path(rgb1), self.device, resize, 0, False)
        with torch.no_grad():
            pred = self.matching({"image0": inp0, "image1": inp1})
        pred = {k: v[0].detach().cpu().numpy() for k, v in pred.items()}
        matches = pred["matches0"]
        valid = matches > -1
        mk0 = rc.scale_keypoints(pred["keypoints0"][valid], scales0)
        mk1 = rc.scale_keypoints(pred["keypoints1"][matches[valid]], scales1)
        return mk0, mk1

    def _teaser_solver(self, A, B):
        """TEASER++ rigid solve, A (3xN, frame i) -> B (3xN, frame i+1)."""
        params = teaserpp_python.RobustRegistrationSolver.Params()
        params.cbar2 = 1.0
        params.noise_bound = float(self.get_parameter("teaser_noise_bound").value)
        params.estimate_scaling = False  # metric RGB-D: scale is known to be 1
        # KCORE_HEU, not PMC_EXACT: PMC_EXACT segfaulted on this data before.
        params.inlier_selection_mode = (
            teaserpp_python.RobustRegistrationSolver.INLIER_SELECTION_MODE.KCORE_HEU
        )
        params.rotation_tim_graph = (
            teaserpp_python.RobustRegistrationSolver.INLIER_GRAPH_FORMULATION.CHAIN
        )
        params.rotation_estimation_algorithm = (
            teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
        )
        params.rotation_gnc_factor = 1.4
        params.rotation_max_iterations = 10000
        params.rotation_cost_threshold = 1e-16
        solver = teaserpp_python.RobustRegistrationSolver(params)
        solver.solve(A, B)
        sol = solver.getSolution()
        if not getattr(sol, "valid", True):
            raise RuntimeError("TEASER++ reported no valid solution")
        T = np.identity(4)
        T[:3, :3] = sol.rotation
        T[:3, 3] = sol.translation
        return T

    def _run_pipeline(self, goal_handle, input_dir, dent_dir, output_dir, ply_path):
        start = time.time()

        if not input_dir.exists():
            raise FileNotFoundError(f"Input directory does not exist: {input_dir}")

        output_dir.mkdir(parents=True, exist_ok=True)

        self._publish_feedback(goal_handle, "discovering_frames", 0.0)
        rgb_paths, depth_paths = discover_frames(input_dir)
        intr = rc.Intrinsics.load(input_dir / "intrinsics.json")
        self.get_logger().info(f"Discovered {len(rgb_paths)} frame pairs in {input_dir}")

        rgb_paths_orig = list(rgb_paths)
        rgb_paths_recon = list(rgb_paths)

        dent_list = sorted(dent_dir.glob("*.json")) if dent_dir.exists() else []
        if not dent_list:
            self.get_logger().warn(
                f"No dent records found in {dent_dir}; running plain reconstruction "
                "with no dent overlay."
            )

        overlay_dir = output_dir / "overlays"
        overlay_gain = float(self.get_parameter("dent_overlay_gain").value)

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
            overlay[y1:y2, x1:x2] = cv2.addWeighted(roi, 1, crop_img, overlay_gain, 0)

            host_stem = Path(rgb_paths_orig[frame_idx]).stem
            overlay_dir.mkdir(parents=True, exist_ok=True)
            out_overlay = overlay_dir / f"{host_stem}_overlay.png"
            if not cv2.imwrite(str(out_overlay), overlay):
                raise RuntimeError(f"Failed to write dent overlay: {out_overlay}")

            rgb_paths_recon[frame_idx] = str(out_overlay)
            dent_hosts.setdefault(frame_idx, []).append(dent_json_path.stem)
            dent_bboxes.setdefault(frame_idx, []).append((x1, y1, x2, y2))

        dent_frame_indices = set(dent_hosts.keys())
        self.get_logger().info(f"Dent host frame indices: {dent_frame_indices}")

        # ---------------- PASS 2: pairwise registration + fusion ----------------
        # World frame of each segment = its first frame. Every frame (including
        # frame 0) gets a pose; a pair that fails its gates starts a NEW segment
        # instead of silently reusing a stale transform.
        cfg = self._reg_config()
        voxel = float(self.get_parameter("voxel_size").value)
        depth_trunc = float(self.get_parameter("depth_trunc").value)
        marker_step = int(self.get_parameter("marker_step_px").value)

        n_frames = len(rgb_paths_recon)
        scene = rc.SegmentedScene(voxel=voxel)
        pair_results = []

        def load_frame(k):
            """(registration cloud, output cloud, raw depth). Registration always uses the
            ORIGINAL frame; on dent host frames the output cloud takes its colour from the
            overlay-tinted image so the dent crop shows up in the exported PLY."""
            depth_raw = depth_read_checked(depth_paths[k])
            cloud_reg = rc.make_cloud(
                imread_checked(rgb_paths_orig[k]), depth_raw, intr, depth_trunc)
            if rgb_paths_recon[k] == rgb_paths_orig[k]:
                cloud_out = cloud_reg
            else:
                cloud_out = rc.make_cloud(
                    imread_checked(rgb_paths_recon[k]), depth_raw, intr, depth_trunc)
            return cloud_reg, cloud_out, depth_raw

        cloud_cur_reg, cloud_cur_out, depth_cur_raw = load_frame(0)
        T_w = np.eye(4)
        scene.new_segment(0)
        scene.add_frame(0, T_w, cloud_cur_out, is_dent_host=(0 in dent_frame_indices))

        n_pairs = max(n_frames - 1, 1)
        for i in range(n_frames - 1):
            self._check_cancel(goal_handle)
            self._publish_feedback(
                goal_handle, f"registering_pair_{i + 1}_of_{n_pairs}",
                0.5 + 0.5 * (i / n_pairs),
            )

            cloud_next_reg, cloud_next_out, depth_next_raw = load_frame(i + 1)
            mk0, mk1 = self._match_frames(rgb_paths_orig[i], rgb_paths_orig[i + 1])

            p0, v0 = rc.backproject_pixels(
                mk0, rc.depth_to_meters(depth_cur_raw, intr.depth_scale), intr, depth_trunc)
            p1, v1 = rc.backproject_pixels(
                mk1, rc.depth_to_meters(depth_next_raw, intr.depth_scale), intr, depth_trunc)
            ok = v0 & v1

            res = rc.register_pair(
                i, p0[ok], p1[ok], cloud_cur_reg, cloud_next_reg, self._teaser_solver, cfg)
            pair_results.append(res)

            if res.ok:
                T_w = T_w @ np.linalg.inv(res.T)
                self.get_logger().info(
                    f"pair {i}->{i + 1} ok ({res.method}, fitness={res.fitness:.2f}, "
                    f"rmse={res.rmse * 1000:.1f} mm, {res.n_corr} corr)")
            else:
                self.get_logger().warn(
                    f"pair {i}->{i + 1} FAILED: {res.reason} -- starting a new segment.")
                scene.new_segment(i + 1)
                T_w = np.eye(4)
            scene.add_frame(i + 1, T_w, cloud_next_out,
                            is_dent_host=((i + 1) in dent_frame_indices))
            cloud_cur_reg, depth_cur_raw = cloud_next_reg, depth_next_raw

        self._check_cancel(goal_handle)
        self._publish_feedback(goal_handle, "marking_dents_in_3d", 0.97)

        dents_marked, dents_skipped = [], []
        for frame_idx in sorted(dent_frame_indices):
            seg, T_wk = scene.locate(frame_idx)
            seg_id = scene.segments.index(seg)
            d_m = rc.depth_to_meters(depth_read_checked(depth_paths[frame_idx]), intr.depth_scale)
            for name, bbox in zip(dent_hosts.get(frame_idx, []), dent_bboxes.get(frame_idx, [])):
                pts = rc.dent_marker_points(
                    bbox, d_m, intr, T_wk, step=marker_step, depth_trunc=depth_trunc)
                if pts.shape[0] == 0:
                    dents_skipped.append({"dent": name, "frame": frame_idx,
                                          "reason": "no valid depth along bbox perimeter"})
                    self.get_logger().warn(
                        f"No valid depth along bbox perimeter for dent {name} "
                        f"(frame {frame_idx}) -- no 3D marker.")
                    continue
                seg.add_marker(pts)
                dents_marked.append({"dent": name, "frame": frame_idx, "segment": seg_id})

        self._check_cancel(goal_handle)
        self._publish_feedback(goal_handle, "saving", 0.99)

        main_seg = scene.largest()
        for j, seg in enumerate(scene.segments):
            out_path = ply_path if seg is main_seg else ply_path.with_name(
                f"{ply_path.stem}_seg{j}{ply_path.suffix}")
            cloud = seg.finalize()
            if not o3d.io.write_point_cloud(str(out_path), cloud):
                raise RuntimeError(f"Failed to write point cloud: {out_path}")

        report = rc.build_quality_report(
            n_frames, pair_results, scene, dents_marked, dents_skipped, cfg,
            extra={"main_segment": scene.segments.index(main_seg),
                   "ply": str(ply_path),
                   "elapsed_s": round(time.time() - start, 1)})
        with open(output_dir / "quality_report.json", "w") as f:
            json.dump(report, f, indent=2)
        self._last_warnings = list(report["warnings"]) + [
            f"dent {d['dent']}: {d['reason']}" for d in dents_skipped]

        self.get_logger().info(
            f"Saved reconstruction to {ply_path} + quality_report.json "
            f"({report['n_pairs_ok']}/{report['n_pairs']} pairs ok, "
            f"{len(scene.segments)} segment(s), took {time.time() - start:.1f}s)")
        self._publish_feedback(goal_handle, "done", 1.0)
        return main_seg


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
