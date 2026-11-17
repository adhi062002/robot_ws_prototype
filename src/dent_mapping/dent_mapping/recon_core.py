"""
recon_core.py

Pure-geometry helpers for the dent reconstruction stage.

This module deliberately imports NOTHING from ROS, torch, SuperGlue or
TEASER++, so it can be unit-tested on any machine (see
test/test_recon_core.py). The ROS node (dent_reconstruction_node.py) injects
the heavy pieces (feature matching, TEASER++) through plain callables.

Conventions
-----------
* Camera frame: x right, y down, z forward (optical frame), metres.
* ``PairResult.T`` maps coordinates of frame i into coordinates of frame i+1
  (p_{i+1} = T @ p_i), same direction as Open3D ``registration_*`` results.
* ``T_w_k`` is the pose of frame k expressed in its segment's first frame
  (p_segment = T_w_k @ p_k). Chain rule: T_w_{k+1} = T_w_k @ inv(T_k).
* ``FLIP`` is applied once, at the very end, so the exported PLY is upright
  in common viewers (camera y points down, viewers expect y up).

Fixes relative to the first version of the node (see git history):
  1. Point clouds are built with the REAL camera intrinsics, not Open3D's
     PrimeSense defaults.
  2. Keypoints are scaled back to full resolution before back-projection, and
     an intrinsics/image resolution mismatch is an error, not silent.
  3. A failed pair never silently reuses a stale transform: the scene is split
     into segments and every pair is gated and reported.
  4. Every frame (including frame 0) has a pose, so dents on any frame get a
     3D marker.
"""

from __future__ import annotations

import json
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import open3d as o3d

FLIP = np.diag([1.0, -1.0, -1.0, 1.0])
DENT_MARKER_COLOR = (1.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Intrinsics
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 0   # 0 = unknown (older runs); size is then taken from the image
    height: int = 0
    depth_scale: float = 1000.0  # raw depth units per metre (RealSense: uint16 mm)

    @classmethod
    def load(cls, path) -> "Intrinsics":
        path = Path(path)
        if not path.exists():
            # A silent fallback to someone else's camera numbers corrupts the
            # whole reconstruction, so fail loudly instead.
            raise FileNotFoundError(f"intrinsics.json not found: {path}")
        with open(path) as f:
            d = json.load(f)
        return cls(
            fx=float(d["fx"]), fy=float(d["fy"]),
            cx=float(d["cx"]), cy=float(d["cy"]),
            width=int(d.get("width", 0)), height=int(d.get("height", 0)),
            depth_scale=float(d.get("depth_scale", 1000.0)),
        )

    def check_image_size(self, width: int, height: int) -> "Intrinsics":
        """Return intrinsics with a validated size; raise if they disagree."""
        if self.width and self.height and (self.width, self.height) != (width, height):
            raise ValueError(
                f"Image is {width}x{height} but intrinsics.json is for "
                f"{self.width}x{self.height}. Re-capture at one resolution or "
                "fix intrinsics.json."
            )
        return Intrinsics(self.fx, self.fy, self.cx, self.cy,
                          width, height, self.depth_scale)

    def to_o3d(self) -> o3d.camera.PinholeCameraIntrinsic:
        return o3d.camera.PinholeCameraIntrinsic(
            int(self.width), int(self.height),
            self.fx, self.fy, self.cx, self.cy)


# ---------------------------------------------------------------------------
# Depth / back-projection
# ---------------------------------------------------------------------------

def depth_to_meters(depth_raw: np.ndarray, depth_scale: float) -> np.ndarray:
    """Raw sensor units -> float32 metres. 0 stays 0 (= invalid)."""
    return (depth_raw.astype(np.float32) / float(depth_scale))


def sample_depth(depth_m: np.ndarray, px: np.ndarray, win: int = 1) -> np.ndarray:
    """Depth at pixel locations, median over a (2*win+1)^2 window of VALID
    pixels, so a keypoint that lands on a 0-depth hole still gets a value
    from its neighbours. Returns 0 where no neighbour is valid."""
    h, w = depth_m.shape[:2]
    u = np.clip(np.rint(px[:, 0]).astype(int), 0, w - 1)
    v = np.clip(np.rint(px[:, 1]).astype(int), 0, h - 1)
    if win <= 0:
        return depth_m[v, u].astype(np.float64)
    stack = []
    for dv in range(-win, win + 1):
        for du in range(-win, win + 1):
            stack.append(depth_m[np.clip(v + dv, 0, h - 1), np.clip(u + du, 0, w - 1)])
    s = np.stack(stack, axis=0).astype(np.float64)
    s[s <= 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        z = np.nanmedian(s, axis=0)
    return np.nan_to_num(z, nan=0.0)


def backproject_pixels(px: np.ndarray, depth_m: np.ndarray, intr: Intrinsics,
                       depth_trunc: Optional[float] = None,
                       win: int = 1) -> Tuple[np.ndarray, np.ndarray]:
    """Pixels (N,2) -> camera-frame 3D points (N,3) plus a validity mask.
    Pixel coordinates must be in the resolution of ``depth_m``."""
    px = np.asarray(px, dtype=np.float64).reshape(-1, 2)
    z = sample_depth(depth_m, px, win=win)
    x = (px[:, 0] - intr.cx) * z / intr.fx
    y = (px[:, 1] - intr.cy) * z / intr.fy
    pts = np.stack([x, y, z], axis=1)
    valid = np.isfinite(z) & (z > 0)
    if depth_trunc is not None:
        valid &= z <= depth_trunc
    return pts, valid


def scale_keypoints(kpts: np.ndarray, scales) -> np.ndarray:
    """Undo the resize SuperGlue's ``read_image`` applied. ``scales`` is the
    (sx, sy) = (orig_w / resized_w, orig_h / resized_h) it returns."""
    return np.asarray(kpts, dtype=np.float64) * np.array([scales[0], scales[1]])


def make_cloud(color_bgr: np.ndarray, depth_raw: np.ndarray, intr: Intrinsics,
               depth_trunc: float = 3.5) -> o3d.geometry.PointCloud:
    """Colored point cloud in the camera frame using the REAL intrinsics."""
    h, w = depth_raw.shape[:2]
    if color_bgr.shape[:2] != (h, w):
        raise ValueError(
            f"color {color_bgr.shape[:2]} and depth {(h, w)} sizes differ; "
            "capture must use aligned depth-to-color at one resolution.")
    intr = intr.check_image_size(w, h)
    rgb = np.ascontiguousarray(color_bgr[:, :, ::-1])
    depth_m = np.ascontiguousarray(depth_to_meters(depth_raw, intr.depth_scale))
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(rgb), o3d.geometry.Image(depth_m),
        depth_scale=1.0, depth_trunc=float(depth_trunc),
        convert_rgb_to_intensity=False)
    return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intr.to_o3d())


def sample_bbox_perimeter(bbox, step: int = 6, size: Optional[Tuple[int, int]] = None):
    """Pixel coords tracing a bbox perimeter, ``step`` px apart. ``size`` =
    (width, height) clamps the box to the image."""
    x1, y1, x2, y2 = [int(round(v)) for v in bbox]
    if size is not None:
        x1, x2 = sorted((max(0, min(x1, size[0] - 1)), max(0, min(x2, size[0] - 1))))
        y1, y2 = sorted((max(0, min(y1, size[1] - 1)), max(0, min(y2, size[1] - 1))))
    pts = []
    for x in range(x1, x2 + 1, step):
        pts.append((x, y1))
        pts.append((x, y2))
    for y in range(y1, y2 + 1, step):
        pts.append((x1, y))
        pts.append((x2, y))
    return np.array(pts, dtype=np.float64) if pts else np.zeros((0, 2))


def dent_marker_points(bbox, depth_m: np.ndarray, intr: Intrinsics, T_w_k: np.ndarray,
                       step: int = 6, depth_trunc: Optional[float] = None) -> np.ndarray:
    """3D outline of a detection bbox, expressed in the segment frame."""
    h, w = depth_m.shape[:2]
    perim = sample_bbox_perimeter(bbox, step=step, size=(w, h))
    if perim.shape[0] == 0:
        return np.zeros((0, 3))
    pts, valid = backproject_pixels(perim, depth_m, intr, depth_trunc=depth_trunc)
    pts = pts[valid]
    if pts.shape[0] == 0:
        return np.zeros((0, 3))
    homog = np.hstack([pts, np.ones((pts.shape[0], 1))])
    return (T_w_k @ homog.T).T[:, :3]


# ---------------------------------------------------------------------------
# Pairwise registration
# ---------------------------------------------------------------------------

@dataclass
class RegConfig:
    icp_voxel: float = 0.02
    icp_max_corr: float = 0.03
    normal_radius: float = 0.04
    max_icp_iterations: int = 200
    min_correspondences: int = 6
    # Acceptance gates for a pair (tune on real data; they go in the report).
    min_icp_fitness: float = 0.30       # fraction of source points with a match
    max_icp_rmse: float = 0.02          # metres, inlier RMSE
    max_correction_m: float = 0.05      # ICP may not drag the initial guess further
    max_correction_deg: float = 10.0
    # Identity-initialised ICP on a flat panel can "succeed" with a wrong
    # in-plane shift, so it is OFF unless you also have a pose prior.
    allow_identity_fallback: bool = False


@dataclass
class PairResult:
    index: int                       # pair (index, index + 1)
    ok: bool
    T: Optional[np.ndarray] = None   # frame index -> frame index+1
    method: str = "none"
    n_corr: int = 0
    fitness: float = 0.0
    rmse: float = float("inf")
    normal_spread: float = float("nan")
    reason: str = ""

    def to_dict(self) -> dict:
        return {
            "pair": [self.index, self.index + 1],
            "ok": bool(self.ok),
            "method": self.method,
            "n_correspondences": int(self.n_corr),
            "icp_fitness": round(float(self.fitness), 4),
            "icp_rmse_m": None if not np.isfinite(self.rmse) else round(float(self.rmse), 5),
            "normal_spread": None if not np.isfinite(self.normal_spread)
            else round(float(self.normal_spread), 4),
            "reason": self.reason,
        }


def is_valid_transform(T: np.ndarray, max_translation: float = 5.0) -> bool:
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        return False
    R = T[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-2):
        return False
    if abs(np.linalg.det(R) - 1.0) > 1e-2:
        return False
    return float(np.linalg.norm(T[:3, 3])) <= max_translation


def transform_delta(T_a: np.ndarray, T_b: np.ndarray) -> Tuple[float, float]:
    """(translation metres, rotation degrees) of T_b relative to T_a."""
    D = T_b @ np.linalg.inv(T_a)
    cos_a = np.clip((np.trace(D[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.linalg.norm(D[:3, 3])), float(np.degrees(np.arccos(cos_a)))


def _prep(cloud: o3d.geometry.PointCloud, cfg: RegConfig) -> o3d.geometry.PointCloud:
    d = cloud.voxel_down_sample(cfg.icp_voxel)
    d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=cfg.normal_radius, max_nn=30))
    d.orient_normals_towards_camera_location(np.zeros(3))  # consistent sign
    return d


def _normal_spread(cloud_down: o3d.geometry.PointCloud) -> float:
    """Second-largest eigenvalue of the normal scatter matrix. ~0 means all
    normals are parallel (a flat wall): ICP cannot constrain in-plane motion,
    so a good fitness there does NOT prove the in-plane position is right."""
    n = np.asarray(cloud_down.normals)
    if n.shape[0] < 3:
        return float("nan")
    ev = np.linalg.eigvalsh(n.T @ n / n.shape[0])
    return float(ev[-2])


def _colored_icp(src, dst, T_init, cfg: RegConfig):
    crit = o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=1e-7, relative_rmse=1e-7,
        max_iteration=cfg.max_icp_iterations)
    return o3d.pipelines.registration.registration_colored_icp(
        src, dst, cfg.icp_max_corr, T_init,
        o3d.pipelines.registration.TransformationEstimationForColoredICP(), crit)


GlobalSolver = Callable[[np.ndarray, np.ndarray], np.ndarray]


def register_pair(index: int,
                  pts_src: np.ndarray, pts_dst: np.ndarray,
                  cloud_src: o3d.geometry.PointCloud,
                  cloud_dst: o3d.geometry.PointCloud,
                  global_solver: Optional[GlobalSolver],
                  cfg: RegConfig,
                  T_prior: Optional[np.ndarray] = None) -> PairResult:
    """Register frame ``index`` -> ``index + 1``.

    pts_src / pts_dst : (N,3) matched 3D points in each camera frame.
    global_solver     : callable (A 3xN, B 3xN) -> 4x4 (e.g. TEASER++).
    T_prior           : optional 4x4 guess (wheel odometry / lidar SLAM) used
                        as an extra ICP seed. Same direction as the result.

    Candidates are tried in order; the first that passes every gate wins.
    Nothing is ever returned as "ok" unless it passed the gates.
    """
    res = PairResult(index=index, ok=False, n_corr=int(pts_src.shape[0]))
    src_d, dst_d = _prep(cloud_src, cfg), _prep(cloud_dst, cfg)
    res.normal_spread = _normal_spread(dst_d)

    candidates: List[Tuple[str, np.ndarray]] = []
    notes: List[str] = []

    if global_solver is None:
        notes.append("no global solver")
    elif pts_src.shape[0] < cfg.min_correspondences:
        notes.append(f"only {pts_src.shape[0]} correspondences "
                     f"(< {cfg.min_correspondences})")
    else:
        try:
            T0 = np.asarray(global_solver(np.ascontiguousarray(pts_src.T),
                                          np.ascontiguousarray(pts_dst.T)))
            if is_valid_transform(T0):
                candidates.append(("global+icp", T0))
            else:
                notes.append("global solver returned an invalid transform")
        except Exception as exc:  # solver failure must not kill the whole run
            notes.append(f"global solver failed: {exc}")

    if T_prior is not None and is_valid_transform(T_prior):
        candidates.append(("prior+icp", np.asarray(T_prior, dtype=np.float64)))
    if cfg.allow_identity_fallback:
        candidates.append(("identity+icp", np.eye(4)))

    if not candidates:
        res.reason = "no usable initial guess: " + "; ".join(notes)
        return res

    for method, T_init in candidates:
        try:
            icp = _colored_icp(src_d, dst_d, T_init, cfg)
        except Exception as exc:
            notes.append(f"{method}: ICP raised {exc}")
            continue
        fit, rmse = float(icp.fitness), float(icp.inlier_rmse)
        T = np.asarray(icp.transformation)
        d_t, d_r = transform_delta(T_init, T)
        if fit > res.fitness:   # keep the best attempt for the report
            res.fitness, res.rmse, res.method = fit, rmse, method
        if fit < cfg.min_icp_fitness:
            notes.append(f"{method}: fitness {fit:.2f} < {cfg.min_icp_fitness}")
        elif rmse > cfg.max_icp_rmse:
            notes.append(f"{method}: rmse {rmse:.4f} m > {cfg.max_icp_rmse}")
        elif d_t > cfg.max_correction_m or d_r > cfg.max_correction_deg:
            notes.append(f"{method}: ICP moved the guess {d_t:.3f} m / {d_r:.1f} deg")
        elif not is_valid_transform(T):
            notes.append(f"{method}: ICP produced an invalid transform")
        else:
            res.ok, res.T, res.method = True, T, method
            res.fitness, res.rmse = fit, rmse
            res.reason = ""
            return res

    res.reason = "; ".join(notes)
    return res


# ---------------------------------------------------------------------------
# Pose chaining and segmented scene
# ---------------------------------------------------------------------------

def chain_poses(n_frames: int, pair_results: List[PairResult]
                ) -> List[Dict[int, np.ndarray]]:
    """Pure helper (used by tests and the report): turn pair results into
    segments of {frame_index: T_w_k}. A failed pair starts a new segment
    instead of reusing a stale transform."""
    by_index = {r.index: r for r in pair_results}
    segments: List[Dict[int, np.ndarray]] = []
    current = {0: np.eye(4)}
    for i in range(n_frames - 1):
        r = by_index.get(i)
        if r is not None and r.ok:
            current[i + 1] = current[i] @ np.linalg.inv(r.T)
        else:
            segments.append(current)
            current = {i + 1: np.eye(4)}
    segments.append(current)
    return segments


class Segment:
    """A run of consecutive frames that were successfully chained."""

    def __init__(self, start: int, voxel: float):
        self.start = start
        self.voxel = voxel
        self.poses: Dict[int, np.ndarray] = {}
        self.pcd = o3d.geometry.PointCloud()       # ordinary frames (re-merged at the end)
        self.host_pcd = o3d.geometry.PointCloud()  # dent-host frames (kept un-merged)
        self.markers: List[np.ndarray] = []

    @property
    def frames(self) -> List[int]:
        return sorted(self.poses)

    def add_frame(self, k: int, T_w_k: np.ndarray, cloud: o3d.geometry.PointCloud,
                  is_dent_host: bool = False) -> None:
        c = cloud.voxel_down_sample(self.voxel)  # new object; original untouched
        c.transform(T_w_k)
        if is_dent_host:
            self.host_pcd += c
        else:
            self.pcd += c
        self.poses[k] = np.array(T_w_k, dtype=np.float64)

    def add_marker(self, pts_seg: np.ndarray) -> None:
        if pts_seg.shape[0]:
            self.markers.append(pts_seg)

    def finalize(self, flip: np.ndarray = FLIP) -> o3d.geometry.PointCloud:
        scene = o3d.geometry.PointCloud()
        if len(self.pcd.points):
            scene += self.pcd.voxel_down_sample(self.voxel)
        scene += self.host_pcd
        for m in self.markers:
            mp = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(m))
            mp.paint_uniform_color(list(DENT_MARKER_COLOR))
            scene += mp   # added after downsampling so it is never merged away
        scene.transform(flip)
        return scene


class SegmentedScene:
    def __init__(self, voxel: float = 0.01):
        self.voxel = voxel
        self.segments: List[Segment] = []
        self._frame_to_seg: Dict[int, int] = {}

    def new_segment(self, start: int) -> Segment:
        seg = Segment(start, self.voxel)
        self.segments.append(seg)
        return seg

    @property
    def current(self) -> Segment:
        return self.segments[-1]

    def add_frame(self, k, T_w_k, cloud, is_dent_host=False) -> None:
        self.current.add_frame(k, T_w_k, cloud, is_dent_host)
        self._frame_to_seg[k] = len(self.segments) - 1

    def locate(self, k: int) -> Optional[Tuple[Segment, np.ndarray]]:
        idx = self._frame_to_seg.get(k)
        if idx is None:
            return None
        seg = self.segments[idx]
        return seg, seg.poses[k]

    def largest(self) -> Segment:
        return max(self.segments, key=lambda s: len(s.poses))


def build_quality_report(n_frames: int, pair_results: List[PairResult],
                         scene: SegmentedScene, dents_marked: List[dict],
                         dents_skipped: List[dict], cfg: RegConfig,
                         extra: Optional[dict] = None) -> dict:
    ok = sum(1 for r in pair_results if r.ok)
    report = {
        "n_frames": n_frames,
        "n_pairs": len(pair_results),
        "n_pairs_ok": ok,
        "n_pairs_failed": len(pair_results) - ok,
        "segments": [
            {"id": i, "first_frame": s.frames[0], "last_frame": s.frames[-1],
             "n_frames": len(s.frames)}
            for i, s in enumerate(scene.segments)
        ],
        "dents_marked": dents_marked,
        "dents_skipped": dents_skipped,
        "pairs": [r.to_dict() for r in pair_results],
        "warnings": [],
        "config": {k: getattr(cfg, k) for k in cfg.__dataclass_fields__},
    }
    if len(scene.segments) > 1:
        report["warnings"].append(
            f"{len(scene.segments)} disconnected segments: registration failed "
            "between them. Each is exported separately; they are NOT aligned "
            "to each other.")
    flat = [r.index for r in pair_results
            if r.ok and np.isfinite(r.normal_spread) and r.normal_spread < 0.02]
    if flat:
        report["warnings"].append(
            f"{len(flat)} pair(s) saw a near-planar scene (pairs {flat[:10]}...): "
            "in-plane drift is not constrained by ICP there. Use odometry/lidar "
            "pose priors for these.")
    if extra:
        report.update(extra)
    return report
