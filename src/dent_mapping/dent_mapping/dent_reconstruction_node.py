--- /home/claude/recon_orig.py	2026-10-21 05:29:07.586279262 +0000
+++ /mnt/user-data/outputs/dent_reconstruction_node.py	2026-10-21 05:29:28.242078003 +0000
@@ -72,6 +72,21 @@
 drawing step, there's no longer a manual/GUI dependency in this node at
 all -- it can run fully unattended as long as YOLO's output matches the
 contract below.
+
+Patch notes (v3)
+----------------
+1. TEASER++ inlier selection: PMC_EXACT -> KCORE_HEU (avoids the segfault).
+2. Open3D clouds are built from the real intrinsics.json (+ image size)
+   instead of PrimeSenseDefault, so clouds and 3D dent markers share one
+   camera model. SuperGlue keypoints are also rescaled from the 640x480
+   network input back to native image coordinates before backprojection.
+3. Dent overlays are written to <output_dir>/overlays/, never into rgb/,
+   and discover_frames ignores any stale *_overlay.png left in rgb/ by
+   older versions, so re-running on the same directory works.
+4. Dents on frame 0 now get a 3D marker (frame 0 had no stored transform).
+5. Registration (SuperGlue + colored ICP) always uses the ORIGINAL frames.
+   The overlay-tinted image is used only to colour the output cloud for
+   dent host frames. Tint strength: parameter dent_overlay_gain (0 = off).
 """
 
 import json
@@ -131,7 +146,10 @@
     scan_dir = Path(scan_dir)
     rgb_dir, depth_dir = scan_dir / "rgb", scan_dir / "depth"
 
-    rgb_files = sorted(rgb_dir.glob("*.png"))
+    # Older versions wrote <stem>_overlay.png into rgb/; ignore any left over.
+    rgb_files = sorted(
+        p for p in rgb_dir.glob("*.png") if not p.stem.endswith("_overlay")
+    )
     depth_files = sorted(depth_dir.glob("*.png"))
 
     if not rgb_files:
@@ -177,6 +195,15 @@
     return np.array(pts, dtype=np.float64) if pts else np.zeros((0, 2))
 
 
+def build_cloud(rgb_path, depth_path, depth_scale, intrinsic):
+    color = o3d.io.read_image(str(rgb_path))
+    depth = o3d.io.read_image(str(depth_path))
+    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
+        color, depth, depth_scale, DEPTH_TRUNC, False
+    )
+    return o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
+
+
 VOXEL_SIZE = 0.1
 DEPTH_TRUNC = 3.5
 
@@ -186,6 +213,7 @@
         super().__init__("dent_reconstruction_node")
 
         self.declare_parameter("dent_subdir", "dent")
+        self.declare_parameter("dent_overlay_gain", 0.5)
 
         self.action_lock = Lock()
         self.active_goal_handle = None
@@ -315,6 +343,9 @@
                 "with no dent overlay."
             )
 
+        overlay_dir = output_dir / "overlays"
+        overlay_gain = float(self.get_parameter("dent_overlay_gain").value)
+
         dent_hosts = {}
         dent_bboxes = {}  # frame_idx -> list of clamped (x1, y1, x2, y2)
 
@@ -393,10 +424,11 @@
 
             overlay = rgb_color.copy()
             roi = overlay[y1:y2, x1:x2]
-            overlay[y1:y2, x1:x2] = cv2.addWeighted(roi, 1, crop_img, 0.5, 0)
+            overlay[y1:y2, x1:x2] = cv2.addWeighted(roi, 1, crop_img, overlay_gain, 0)
 
             host_stem = Path(rgb_paths_orig[frame_idx]).stem
-            out_overlay = Path(rgb_paths_orig[frame_idx]).parent / f"{host_stem}_overlay.png"
+            overlay_dir.mkdir(parents=True, exist_ok=True)
+            out_overlay = overlay_dir / f"{host_stem}_overlay.png"
             if not cv2.imwrite(str(out_overlay), overlay):
                 raise RuntimeError(f"Failed to write dent overlay: {out_overlay}")
 
@@ -421,8 +453,12 @@
                 0.5 + 0.5 * (i / n_pairs),
             )
 
-            rgb0 = Path(rgb_paths_recon[i])
-            rgb1 = Path(rgb_paths_recon[i + 1])
+            # Registration always uses the original frames; the overlay-tinted
+            # versions (dent host frames only) just colour the output cloud.
+            rgb0 = Path(rgb_paths_orig[i])
+            rgb1 = Path(rgb_paths_orig[i + 1])
+            rgb0_out = Path(rgb_paths_recon[i])
+            rgb1_out = Path(rgb_paths_recon[i + 1])
             depth0 = Path(depth_paths[i])
             depth1 = Path(depth_paths[i + 1])
 
@@ -442,8 +478,10 @@
             kpts0, kpts1 = pred["keypoints0"], pred["keypoints1"]
             matches, conf = pred["matches0"], pred["matching_scores0"]
             valid = matches > -1
-            mkpts0 = kpts0[valid]
-            mkpts1 = kpts1[matches[valid]]
+            # Keypoints are in the 640x480 network-input frame; depth and
+            # intrinsics are in native resolution (scales == (1, 1) at 640x480).
+            mkpts0 = kpts0[valid] * np.asarray(scales0, dtype=np.float64)
+            mkpts1 = kpts1[matches[valid]] * np.asarray(scales1, dtype=np.float64)
 
             pts3d_0_all, valid0 = backproject(mkpts0, d0, fx, fy, cx, cy)
             pts3d_1_all, valid1 = backproject(mkpts1, d1, fx, fy, cx, cy)
@@ -466,7 +504,7 @@
             solver_params.noise_bound = VOXEL_SIZE
             solver_params.estimate_scaling = True
             solver_params.inlier_selection_mode = (
-                teaserpp_python.RobustRegistrationSolver.INLIER_SELECTION_MODE.PMC_EXACT
+                teaserpp_python.RobustRegistrationSolver.INLIER_SELECTION_MODE.KCORE_HEU
             )
             solver_params.rotation_tim_graph = (
                 teaserpp_python.RobustRegistrationSolver.INLIER_GRAPH_FORMULATION.CHAIN
@@ -484,26 +522,25 @@
             T_teaser[:3, :3] = solution.rotation
             T_teaser[:3, 3] = solution.translation
 
-            color0_o3d = o3d.io.read_image(str(rgb0))
-            depth0_o3d = o3d.io.read_image(str(depth0))
-            color1_o3d = o3d.io.read_image(str(rgb1))
-            depth1_o3d = o3d.io.read_image(str(depth1))
-
-            rgbd0 = o3d.geometry.RGBDImage.create_from_color_and_depth(
-                color0_o3d, depth0_o3d, depth_scale, DEPTH_TRUNC, False
-            )
-            rgbd1 = o3d.geometry.RGBDImage.create_from_color_and_depth(
-                color1_o3d, depth1_o3d, depth_scale, DEPTH_TRUNC, False
-            )
-
+            h_img, w_img = d0_raw.shape[:2]
             intrinsic = o3d.camera.PinholeCameraIntrinsic(
-                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault
+                int(w_img), int(h_img), fx, fy, cx, cy
             )
-            compcd0 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd0, intrinsic)
-            compcd1 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd1, intrinsic)
 
-            compcd0_down = compcd0.voxel_down_sample(0.02)
-            compcd1_down = compcd1.voxel_down_sample(0.02)
+            # Registration clouds: original frames only.
+            reg_pcd0 = build_cloud(rgb0, depth0, depth_scale, intrinsic)
+            reg_pcd1 = build_cloud(rgb1, depth1, depth_scale, intrinsic)
+            compcd0_down = reg_pcd0.voxel_down_sample(0.02)
+            compcd1_down = reg_pcd1.voxel_down_sample(0.02)
+
+            # Output clouds: overlay-tinted colour on dent host frames. The
+            # downsampled copies above are independent, so transforming these
+            # later does not touch the registration clouds.
+            compcd0 = reg_pcd0 if rgb0_out == rgb0 else build_cloud(
+                rgb0_out, depth0, depth_scale, intrinsic)
+            compcd1 = reg_pcd1 if rgb1_out == rgb1 else build_cloud(
+                rgb1_out, depth1, depth_scale, intrinsic)
+
             compcd0_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.04, max_nn=30))
             compcd1_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(0.04, max_nn=30))
 
@@ -518,6 +555,8 @@
             FLIP = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
 
             if i == 0:
+                if 0 in dent_frame_indices:
+                    dent_transforms[0] = np.array(FLIP) @ result_icp.transformation
                 compcd0.transform(result_icp.transformation)
                 compcd0.transform(FLIP)
                 compcd1.transform(FLIP)
