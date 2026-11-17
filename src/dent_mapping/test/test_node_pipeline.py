"""Integration test: runs the REAL DentReconstructionNode._run_pipeline with
ROS / torch / SuperGlue / TEASER++ stubbed out, on synthetic frames with known
poses. Checks the glue code: file discovery, pass 1 overlay, pose chaining,
dent markers (incl. frame 0), PLY + quality_report.json, and failure splitting.

Run:  cd src/dent_mapping && python -m pytest test/test_node_pipeline.py -q
"""
import importlib
import json
import logging
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import open3d as o3d
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))                       # test_recon_core helpers
sys.path.insert(0, str(HERE.parent))                # package root (dent_mapping/)
import test_recon_core as T  # noqa: E402


def _stub_heavy_modules():
    class _Node:
        pass

    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    anything = lambda *a, **k: None  # noqa: E731
    mod("teaserpp_python")
    mod("torch", cuda=SimpleNamespace(is_available=lambda: False), no_grad=anything)
    mod("rclpy")
    mod("rclpy.action", ActionServer=anything, CancelResponse=SimpleNamespace(),
        GoalResponse=SimpleNamespace())
    mod("rclpy.callback_groups", ReentrantCallbackGroup=anything)
    mod("rclpy.executors", MultiThreadedExecutor=anything)
    mod("rclpy.node", Node=_Node)
    mod("models")
    mod("models.matching", Matching=anything)
    mod("models.utils", read_image=anything)
    mod("dent_mapping_interfaces")
    mod("dent_mapping_interfaces.action", ReconstructScan=SimpleNamespace())


@pytest.fixture(scope="module")
def node_mod():
    _stub_heavy_modules()
    sys.modules.pop("dent_mapping.dent_reconstruction_node", None)
    return importlib.import_module("dent_mapping.dent_reconstruction_node")


PARAMS = {
    "voxel_size": 0.01, "depth_trunc": 3.5, "marker_step_px": 6,
    "superglue_resize": [640, 480], "teaser_noise_bound": 0.02,
    "min_correspondences": 6, "icp_voxel": 0.02, "icp_max_corr": 0.03,
    "dent_overlay_gain": 0.5, "min_icp_fitness": 0.30, "max_icp_rmse": 0.02,
    "max_correction_m": 0.05, "max_correction_deg": 10.0,
}


def make_node(node_mod, matcher):
    n = object.__new__(node_mod.DentReconstructionNode)
    n.get_logger = lambda: logging.getLogger("node-test")
    n.get_parameter = lambda name: SimpleNamespace(value=PARAMS[name])
    n._publish_feedback = lambda *a, **k: None
    n._check_cancel = lambda *a, **k: None
    n._last_warnings = []
    n.device = "cpu"
    n._match_frames = matcher
    n._teaser_solver = T.ransac_solver
    return n


def write_scan(root, poses):
    (root / "rgb").mkdir(parents=True)
    (root / "depth").mkdir()
    frames = []
    for k, pose in enumerate(poses):
        bgr, depth = T.render(pose)
        cv2.imwrite(str(root / "rgb" / f"{k:06d}.png"), bgr)
        cv2.imwrite(str(root / "depth" / f"{k:06d}.png"), depth)
        frames.append((bgr, depth))
    (root / "intrinsics.json").write_text(json.dumps(
        {"fx": T.INTR.fx, "fy": T.INTR.fy, "cx": T.INTR.cx, "cy": T.INTR.cy,
         "width": T.W, "height": T.H}))
    return frames


def write_dent(root, name, frame_idx, bbox, frames):
    (root / "dent").mkdir(exist_ok=True)
    x1, y1, x2, y2 = bbox
    crop = frames[frame_idx][0][y1:y2, x1:x2]
    cv2.imwrite(str(root / "dent" / f"{name}.png"), crop)
    (root / "dent" / f"{name}.json").write_text(json.dumps(
        {"frame": f"{frame_idx:06d}.png", "bbox": list(bbox), "confidence": 0.9}))


def gt_marker_in_seg0(poses, frames, k, bbox):
    d = rc_depth(frames[k][1])
    local, ok = T.rc.backproject_pixels(
        T.rc.sample_bbox_perimeter(bbox, size=(T.W, T.H)), d, T.INTR)
    return (np.linalg.inv(poses[0]) @ poses[k] @ np.hstack(
        [local[ok], np.ones((ok.sum(), 1))]).T).T[:, :3]


def rc_depth(raw):
    return T.rc.depth_to_meters(raw, 1000.0)


def red_points_unflipped(ply_path):
    pc = o3d.io.read_point_cloud(str(ply_path))
    pts, cols = np.asarray(pc.points), np.asarray(pc.colors)
    red = pts[np.all(np.isclose(cols, [1, 0, 0], atol=0.02), axis=1)]
    return red * np.array([1, -1, -1])              # undo the viewer flip


def test_full_pipeline_marks_dents_on_frame0_and_last_frame(tmp_path, node_mod):
    poses = [T.make_pose(), T.make_pose(0.05, -0.02, 0.01, 2, 1),
             T.make_pose(0.10, -0.03, 0.02, 4, 2), T.make_pose(0.15, -0.05, 0.03, 5, 2)]
    scan = tmp_path / "scan"
    frames = write_scan(scan, poses)
    bboxes = {0: (100, 100, 180, 160), 3: (280, 200, 360, 260)}
    for j, (k, b) in enumerate(bboxes.items()):
        write_dent(scan, f"dent_{j:03d}", k, b, frames)
    # a stale overlay left in rgb/ by an older version must be ignored, not treated as a frame
    cv2.imwrite(str(scan / "rgb" / "000001_overlay.png"), frames[1][0])

    seen_paths = []

    def matcher(rgb0, rgb1):
        seen_paths.extend([str(rgb0), str(rgb1)])
        i = int(Path(rgb0).stem)               # raises if an overlay path sneaks in
        return T.make_pixel_matches(poses[i], poses[i + 1], frames[i][1], frames[i + 1][1],
                                    seed=100 + i)

    node = make_node(node_mod, matcher)
    out = tmp_path / "out"
    ply = out / "reconstruction.ply"
    node._run_pipeline(None, scan, scan / "dent", out, ply)

    report = json.loads((out / "quality_report.json").read_text())
    assert report["n_frames"] == 4, "stale *_overlay.png in rgb/ was counted as a frame"
    assert report["n_pairs_ok"] == 3 and report["n_pairs_failed"] == 0, report["pairs"]
    # registration saw ONLY original frames; overlays live in out/overlays, never in rgb/
    assert seen_paths and not any("overlay" in p for p in seen_paths)
    assert sorted(f.name for f in (out / "overlays").glob("*.png")) == [
        "000000_overlay.png", "000003_overlay.png"]
    stale = sorted(f.name for f in (scan / "rgb").glob("*overlay*.png"))
    assert stale == ["000001_overlay.png"], "node must not write overlays into rgb/"
    assert len(report["segments"]) == 1
    assert sorted(d["frame"] for d in report["dents_marked"]) == [0, 3]   # frame 0 included
    assert report["dents_skipped"] == []

    red = red_points_unflipped(ply)
    assert red.shape[0] > 40
    gt = np.vstack([gt_marker_in_seg0(poses, frames, k, b) for k, b in bboxes.items()])
    # every GT marker point must have a produced red point within 1 cm
    from scipy.spatial import cKDTree
    dist, _ = cKDTree(red).query(gt)
    assert np.percentile(dist, 95) < 0.01, np.percentile(dist, 95)
    # no registration problems; the only warning allowed is the near-flat-scene one
    assert not any("disconnected" in w for w in node._last_warnings)
    assert any("near-planar" in w for w in report["warnings"])   # this synthetic wall is ~flat


def test_failed_pair_gives_two_segments_report_and_separate_ply(tmp_path, node_mod):
    poses = [T.make_pose(), T.make_pose(0.05, 0, 0.01, 2), T.make_pose(0.10, 0.0, 0.02, 3),
             T.make_pose(1.2, 0.6, 0.0, 28)]       # last frame: practically no overlap
    scan = tmp_path / "scan"
    frames = write_scan(scan, poses)
    write_dent(scan, "dent_000", 3, (280, 200, 360, 260), frames)

    def matcher(rgb0, rgb1):
        i = int(Path(rgb0).stem)
        if i == 2:                                  # garbage matches for the broken pair
            rng = np.random.default_rng(7)
            return (np.column_stack([rng.uniform(20, 600, 60), rng.uniform(20, 440, 60)]),
                    np.column_stack([rng.uniform(20, 600, 60), rng.uniform(20, 440, 60)]))
        return T.make_pixel_matches(poses[i], poses[i + 1], frames[i][1], frames[i + 1][1],
                                    seed=200 + i)

    node = make_node(node_mod, matcher)
    out = tmp_path / "out"
    ply = out / "reconstruction.ply"
    node._run_pipeline(None, scan, scan / "dent", out, ply)

    report = json.loads((out / "quality_report.json").read_text())
    assert report["n_pairs_ok"] == 2 and report["n_pairs_failed"] == 1
    assert [s["n_frames"] for s in report["segments"]] == [3, 1]
    assert report["main_segment"] == 0
    assert any("disconnected" in w for w in report["warnings"])
    assert (out / "reconstruction.ply").exists() and (out / "reconstruction_seg1.ply").exists()
    assert report["dents_marked"][0]["segment"] == 1      # dent lives in the 2nd segment
    assert any("disconnected" in w for w in node._last_warnings)
