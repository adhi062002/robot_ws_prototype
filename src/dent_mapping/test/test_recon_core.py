"""Tests for recon_core using synthetic RGB-D frames with known poses.

Run:  cd src/dent_mapping && python -m pytest test/test_recon_core.py -q
(needs only numpy + open3d; no ROS, torch, SuperGlue or TEASER++).
"""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "dent_mapping"))
import recon_core as rc  # noqa: E402

W, H = 640, 480
INTR = rc.Intrinsics(fx=600.0, fy=598.0, cx=310.0, cy=245.0, width=W, height=H)
Z0 = 1.0


# ----------------------------- synthetic world -----------------------------

def surface_h(x, y):
    return (0.03 * np.sin(6 * x) * np.cos(5 * y)
            + 0.02 * np.exp(-((x - 0.1) ** 2 + (y + 0.05) ** 2) / 0.01))


def texture(x, y):
    r = 0.5 + 0.5 * np.sin(37 * x + 11 * y)
    g = 0.5 + 0.5 * np.sin(23 * y - 17 * x + 1.0)
    b = 0.5 + 0.5 * np.sin(41 * x * y + 7 * x)
    return np.stack([b, g, r], axis=-1)  # BGR like cv2


def make_pose(tx=0.0, ty=0.0, tz=0.0, yaw_deg=0.0, pitch_deg=0.0):
    """Camera pose in the world (cam -> world)."""
    cy_, sy_ = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    cp, sp = np.cos(np.radians(pitch_deg)), np.sin(np.radians(pitch_deg))
    Ry = np.array([[cy_, 0, sy_], [0, 1, 0], [-sy_, 0, cy_]])
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    T = np.eye(4)
    T[:3, :3] = Ry @ Rx
    T[:3, 3] = [tx, ty, tz]
    return T


def render(T_w_c, intr=INTR):
    """Ray-cast the synthetic surface Z = Z0 + h(X, Y). Returns (bgr, depth_u16_mm)."""
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    d_c = np.stack([(u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy,
                    np.ones_like(u, dtype=float)], axis=-1).reshape(-1, 3)
    R, o = T_w_c[:3, :3], T_w_c[:3, 3]
    d_w = d_c @ R.T
    lo, hi = np.full(d_w.shape[0], 0.2), np.full(d_w.shape[0], 3.0)
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        p = o + mid[:, None] * d_w
        g = p[:, 2] - Z0 - surface_h(p[:, 0], p[:, 1])
        hi = np.where(g > 0, mid, hi)
        lo = np.where(g > 0, lo, mid)
    t = 0.5 * (lo + hi)                      # = camera z-depth (d_c has z = 1)
    p = o + t[:, None] * d_w
    bgr = (texture(p[:, 0], p[:, 1]) * 255).astype(np.uint8).reshape(H, W, 3)
    depth = np.rint(t * 1000.0).astype(np.uint16).reshape(H, W)
    return bgr, depth


def gt_pair_T(T_w_c0, T_w_c1):
    """Maps frame-0 coords -> frame-1 coords."""
    return np.linalg.inv(T_w_c1) @ T_w_c0


def make_pixel_matches(T0, T1, depth0, depth1, n=400, noise_px=0.7,
                       outlier_frac=0.2, seed=0):
    """Mimic SuperGlue output: matched pixel coords (frame 0, frame 1) with
    pixel noise and a fraction of outliers."""
    rng = np.random.default_rng(seed)
    px0 = np.column_stack([rng.uniform(20, W - 20, n), rng.uniform(20, H - 20, n)])
    d0 = rc.depth_to_meters(depth0, 1000.0)
    p0, ok0 = rc.backproject_pixels(px0, d0, INTR)
    p_w = (T0 @ np.hstack([p0, np.ones((n, 1))]).T).T
    p_1 = (np.linalg.inv(T1) @ p_w.T).T[:, :3]
    z = np.where(p_1[:, 2] > 1e-6, p_1[:, 2], np.nan)
    px1 = np.column_stack([INTR.fx * p_1[:, 0] / z + INTR.cx,
                           INTR.fy * p_1[:, 1] / z + INTR.cy])
    px1 = px1 + rng.normal(0, noise_px, px1.shape)
    out = rng.random(n) < outlier_frac
    px1[out] = np.column_stack([rng.uniform(0, W, out.sum()), rng.uniform(0, H, out.sum())])
    inb = (np.isfinite(px1).all(axis=1) & (px1[:, 0] > 2) & (px1[:, 0] < W - 3)
           & (px1[:, 1] > 2) & (px1[:, 1] < H - 3))
    keep = inb & ok0
    return px0[keep], px1[keep]


def make_correspondences(T0, T1, depth0, depth1, **kw):
    px0, px1 = make_pixel_matches(T0, T1, depth0, depth1, **kw)
    d0 = rc.depth_to_meters(depth0, 1000.0)
    d1 = rc.depth_to_meters(depth1, 1000.0)
    pts0, v0 = rc.backproject_pixels(px0, d0, INTR)
    pts1, v1 = rc.backproject_pixels(px1, d1, INTR)
    m = v0 & v1
    return pts0[m], pts1[m]


def kabsch(A, B):
    ca, cb = A.mean(0), B.mean(0)
    Hm = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(Hm)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, cb - R @ ca
    return T


def ransac_solver(A3xN, B3xN, iters=300, thresh=0.02, min_inliers=12, seed=1):
    """Stand-in for TEASER++ in tests: RANSAC + Kabsch (maps A -> B)."""
    A, B = A3xN.T, B3xN.T
    rng = np.random.default_rng(seed)
    best, best_n = None, 0
    for _ in range(iters):
        idx = rng.choice(len(A), 3, replace=False)
        T = kabsch(A[idx], B[idx])
        err = np.linalg.norm((T[:3, :3] @ A.T).T + T[:3, 3] - B, axis=1)
        n = int((err < thresh).sum())
        if n > best_n:
            best, best_n = err < thresh, n
    if best is None or best_n < min_inliers:
        raise RuntimeError(f"only {best_n} inliers")
    return kabsch(A[best], B[best])


def clouds_for(frames):
    return [rc.make_cloud(bgr, dep, INTR) for bgr, dep in frames]


# ----------------------------------- tests ----------------------------------

def test_intrinsics_roundtrip_and_size_check(tmp_path):
    p = tmp_path / "intrinsics.json"
    p.write_text(json.dumps({"fx": 600.0, "fy": 598.0, "cx": 310.0, "cy": 245.0,
                             "width": W, "height": H}))
    i = rc.Intrinsics.load(p)
    assert (i.fx, i.cx, i.width, i.height) == (600.0, 310.0, W, H)
    with pytest.raises(ValueError):
        i.check_image_size(1280, 720)           # silent resolution mismatch is an error
    with pytest.raises(FileNotFoundError):
        rc.Intrinsics.load(tmp_path / "missing.json")  # no silent default camera


def test_cloud_uses_real_intrinsics_not_primesense_default():
    """Bug 1: a flat 1 m wall must span x in [(0-cx)/fx, (W-1-cx)/fx]."""
    bgr = np.full((H, W, 3), 128, np.uint8)
    depth = np.full((H, W), 1000, np.uint16)
    pts = np.asarray(rc.make_cloud(bgr, depth, INTR).points)
    assert pts[:, 0].min() == pytest.approx((0 - INTR.cx) / INTR.fx, abs=1e-4)
    assert pts[:, 0].max() == pytest.approx((W - 1 - INTR.cx) / INTR.fx, abs=1e-4)
    assert pts[:, 1].max() == pytest.approx((H - 1 - INTR.cy) / INTR.fy, abs=1e-4)
    assert np.allclose(pts[:, 2], 1.0)
    # PrimeSense default (fx=525, cx=319.5) would put x_min at -0.6086, not -0.5167
    assert abs(pts[:, 0].min() - (-319.5 / 525.0)) > 0.05


def test_scale_keypoints_restores_full_resolution():
    k = np.array([[320.0, 240.0], [10.0, 20.0]])
    out = rc.scale_keypoints(k, (2.0, 1.5))     # captured 1280x720, matched at 640x480
    assert np.allclose(out, [[640.0, 360.0], [20.0, 30.0]])


def test_backproject_recovers_known_points_and_handles_holes():
    depth = np.full((H, W), 1.5, np.float32)
    depth[100, 200] = 0.0                       # hole under the keypoint
    pts, valid = rc.backproject_pixels(np.array([[200.0, 100.0]]), depth, INTR)
    assert valid[0] and pts[0, 2] == pytest.approx(1.5)   # filled from neighbours
    assert pts[0, 0] == pytest.approx((200 - INTR.cx) * 1.5 / INTR.fx)
    _, valid0 = rc.backproject_pixels(np.array([[200.0, 100.0]]), depth, INTR, win=0)
    assert not valid0[0]


def test_register_pair_recovers_ground_truth_pose():
    T0, T1 = make_pose(), make_pose(0.06, -0.03, 0.02, yaw_deg=3, pitch_deg=2)
    (b0, d0), (b1, d1) = render(T0), render(T1)
    c0, c1 = clouds_for([(b0, d0), (b1, d1)])
    p0, p1 = make_correspondences(T0, T1, d0, d1)
    res = rc.register_pair(0, p0, p1, c0, c1, ransac_solver, rc.RegConfig())
    assert res.ok, res.reason
    dt, dr = rc.transform_delta(gt_pair_T(T0, T1), res.T)
    assert dt < 0.004 and dr < 0.3, (dt, dr)
    assert res.fitness > 0.5


def test_pose_chain_matches_ground_truth_and_frame0_dent_is_marked():
    """Bug 4: every frame, including 0, has a pose and gets a dent marker."""
    poses = [make_pose(), make_pose(0.05, -0.02, 0.01, 2, 1),
             make_pose(0.10, -0.03, 0.02, 4, 2)]
    frames = [render(T) for T in poses]
    clouds = clouds_for(frames)
    results = []
    for i in range(2):
        p0, p1 = make_correspondences(poses[i], poses[i + 1], frames[i][1],
                                      frames[i + 1][1], seed=10 + i)
        results.append(rc.register_pair(i, p0, p1, clouds[i], clouds[i + 1],
                                        ransac_solver, rc.RegConfig()))
    assert all(r.ok for r in results), [r.reason for r in results]

    scene = rc.SegmentedScene(voxel=0.01)
    scene.new_segment(0)
    T_w = np.eye(4)
    scene.add_frame(0, T_w, clouds[0], is_dent_host=True)   # frame 0 hosts a dent
    for i, r in enumerate(results):
        T_w = T_w @ np.linalg.inv(r.T)
        scene.add_frame(i + 1, T_w, clouds[i + 1], is_dent_host=(i + 1 == 2))

    chained = rc.chain_poses(3, results)[0]
    for k in range(3):   # chain_poses must agree with ground truth (frame 0 = world)
        gt_pose = np.linalg.inv(poses[0]) @ poses[k]
        dt, dr = rc.transform_delta(gt_pose, chained[k])
        assert dt < 0.008 and dr < 0.5, (k, dt, dr)

    bbox = (280, 200, 360, 260)
    for k in (0, 2):
        seg, T_wk = scene.locate(k)
        m = rc.dent_marker_points(bbox, rc.depth_to_meters(frames[k][1], 1000.0), INTR, T_wk)
        assert m.shape[0] > 10
        # ground truth: same pixels back-projected, moved with the TRUE pose
        gt_local, ok = rc.backproject_pixels(
            rc.sample_bbox_perimeter(bbox, size=(W, H)),
            rc.depth_to_meters(frames[k][1], 1000.0), INTR)
        gt = (np.linalg.inv(poses[0]) @ poses[k] @ np.hstack(
            [gt_local[ok], np.ones((ok.sum(), 1))]).T).T[:, :3]
        assert np.abs(m - gt).max() < 0.008
        seg.add_marker(m)

    out = scene.segments[0].finalize()
    colors = np.asarray(out.colors)
    assert len(out.points) > 1000 and colors.shape[0] == len(out.points)
    assert (np.all(colors == [1, 0, 0], axis=1)).sum() > 20     # red markers present


def test_failed_pair_splits_segments_instead_of_reusing_stale_transform():
    """Bug 4: a bad pair must be reported and must break the chain."""
    poses = [make_pose(), make_pose(0.05, 0, 0.01, 2), make_pose(0.9, 0.5, 0.0, 25)]
    frames = [render(T) for T in poses]
    clouds = clouds_for(frames)
    p0, p1 = make_correspondences(poses[0], poses[1], frames[0][1], frames[1][1])
    good = rc.register_pair(0, p0, p1, clouds[0], clouds[1], ransac_solver, rc.RegConfig())
    rng = np.random.default_rng(5)                 # garbage matches
    bad = rc.register_pair(1, rng.normal(size=(60, 3)), rng.normal(size=(60, 3)),
                           clouds[1], clouds[2], ransac_solver, rc.RegConfig())
    assert good.ok and not bad.ok and bad.reason
    segs = rc.chain_poses(3, [good, bad])
    assert [sorted(s) for s in segs] == [[0, 1], [2]]
    assert np.allclose(segs[1][2], np.eye(4))      # new segment, not a stale pose

    scene = rc.SegmentedScene()
    scene.new_segment(0)
    scene.add_frame(0, np.eye(4), clouds[0])
    scene.add_frame(1, segs[0][1], clouds[1])
    scene.new_segment(2)
    scene.add_frame(2, np.eye(4), clouds[2])
    report = rc.build_quality_report(3, [good, bad], scene, [], [], rc.RegConfig())
    assert report["n_pairs_failed"] == 1 and len(report["segments"]) == 2
    assert any("disconnected" in w for w in report["warnings"])
    json.dumps(report)                              # must be JSON-serialisable


def test_identity_fallback_is_off_by_default_and_prior_is_used():
    T0, T1 = make_pose(), make_pose(0.05, -0.02, 0.01, 2, 1)
    (b0, d0), (b1, d1) = render(T0), render(T1)
    c0, c1 = clouds_for([(b0, d0), (b1, d1)])
    cfg = rc.RegConfig()
    nothing = rc.register_pair(0, np.zeros((0, 3)), np.zeros((0, 3)), c0, c1, ransac_solver, cfg)
    assert not nothing.ok and "no usable initial guess" in nothing.reason
    prior = gt_pair_T(T0, T1)
    prior_noisy = prior.copy()
    prior_noisy[:3, 3] += [0.01, -0.01, 0.005]      # odometry-grade error
    with_prior = rc.register_pair(0, np.zeros((0, 3)), np.zeros((0, 3)), c0, c1,
                                  None, cfg, T_prior=prior_noisy)
    assert with_prior.ok and with_prior.method == "prior+icp"
    dt, _ = rc.transform_delta(prior, with_prior.T)
    assert dt < 0.004


def test_normal_spread_flags_flat_wall_but_not_a_corner():
    """ICP cannot fix in-plane drift on a flat wall; the report must say so."""
    cfg = rc.RegConfig()
    bgr = np.full((H, W, 3), 120, np.uint8)
    flat = rc.make_cloud(bgr, np.full((H, W), 1000, np.uint16), INTR)
    # two walls meeting at an angle: depth ramps up on the left half, down on the right
    cols = np.abs(np.arange(W) - W / 2) * 1.2 + 800
    corner = rc.make_cloud(bgr, np.tile(cols.astype(np.uint16), (H, 1)), INTR)
    spread_flat = rc._normal_spread(rc._prep(flat, cfg))
    spread_corner = rc._normal_spread(rc._prep(corner, cfg))
    assert spread_flat < 0.005
    assert spread_corner > 0.05
