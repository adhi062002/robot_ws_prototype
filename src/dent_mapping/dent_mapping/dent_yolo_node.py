#!/usr/bin/env python3
"""
dent_yolo_node.py

Stage 1 of the dent-mapping pipeline:
    CAPTURE -> YOLO (this node) -> RECONSTRUCTION -> POLISHING

Reads a capture run (rgb/, depth/, intrinsics.json), runs every frame
through an Ultralytics YOLO model, and writes a self-contained output
folder that the reconstruction node (dent_reconstruction_node.py) already
knows how to consume directly:

    <output_dir>/
      rgb/<stem>.png          -- every frame, copied through unchanged
                                 (detected or not -- same filenames/order
                                 as the capture run)
      depth/<stem>.png        -- copied through unchanged from capture
      intrinsics.json         -- copied through unchanged from capture
      dent/
        <stem>_000.png        -- cropped bbox region, detection #0 on
                                  that frame (further detections on the
                                  same frame are _001, _002, ...)
        <stem>_000.json        -- {"frame": "<stem>.png",
                                    "bbox": [x1, y1, x2, y2],
                                    "confidence": 0.83,
                                    "class": "dent"}

bbox is in pixel coordinates of the full-resolution frame (Ultralytics
already rescales detections back to the original image size, so no
extra scaling is needed here).

Input resolution
-----------------
If the action goal's input_dir is left empty, this node auto-discovers
the latest timestamped run under ~/datasets (or the configured
datasets_root param) -- this is the ONLY node in the pipeline that does
that auto-discovery; the reconstruction node always receives an explicit
input_dir instead (normally this node's output_dir).

Threading / execution model
----------------------------
Same pattern as the other two nodes in this package: execute_callback is
a plain blocking function (NOT async def -- rclpy's action executor does
not run a real asyncio loop). Only one detection run is allowed at a time
(enforced in goal_callback) since the YOLO model instance and GPU aren't
meant to be shared across concurrent goals. The model loads once in
__init__, not per-goal. Cancellation is checked between frames; feedback
(stage + progress + frames_processed/total_frames) is published per
frame.
"""

import json
import shutil
import time
from pathlib import Path
from threading import Lock

import cv2
import torch

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ultralytics import YOLO

from dent_mapping_interfaces.action import DetectDents


class GoalCancelled(Exception):
    """Raised internally to unwind out of the pipeline on cancellation."""


def find_latest_run(datasets_root):
    """Pick the most recent timestamped capture run under datasets_root.

    Capture runs are named <timestamp> or <timestamp>_NN (see
    RealsenseFrameCaptureNode._create_run), which sorts correctly as
    plain strings -- no need to parse the timestamp.
    """
    datasets_root = Path(datasets_root)
    if not datasets_root.exists():
        raise FileNotFoundError(f"Datasets root does not exist: {datasets_root}")

    candidates = [
        p for p in datasets_root.iterdir()
        if p.is_dir() and (p / "rgb").is_dir() and (p / "depth").is_dir()
    ]
    if not candidates:
        raise FileNotFoundError(
            f"No capture runs (with rgb/ and depth/ subfolders) found under {datasets_root}"
        )

    return sorted(candidates, key=lambda p: p.name)[-1]


class DentYoloNode(Node):
    def __init__(self):
        super().__init__("dent_yolo_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("datasets_root", str(Path.home() / "datasets"))
        self.declare_parameter("output_subdir", "yolo_output")
        self.declare_parameter("default_confidence", 0.25)
        self.declare_parameter("dent_class_name", "dent")

        model_path = self.get_parameter("model_path").value
        if not model_path:
            raise RuntimeError(
                "The 'model_path' parameter is required (path to best.pt). "
                "Set it via --ros-args -p model_path:=/path/to/best.pt "
                "or in your launch file."
            )
        model_path = Path(model_path).expanduser()
        if not model_path.exists():
            raise FileNotFoundError(f"YOLO weights not found: {model_path}")

        self.action_lock = Lock()
        self.active_goal_handle = None
        self.io_callback_group = ReentrantCallbackGroup()

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Loading YOLO model from {model_path} on device={self.device}...")
        self.model = YOLO(str(model_path))
        self.model.to(self.device)
        self.get_logger().info("YOLO model loaded.")

        self.action_server = ActionServer(
            self,
            DetectDents,
            "detect_dents",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.io_callback_group,
        )

        self.get_logger().info(
            "Dent YOLO detection action server ready. Action: /detect_dents"
        )

    # ------------------------------------------------------------------
    # ROS 2 Action handling
    # ------------------------------------------------------------------

    def goal_callback(self, goal_request):
        with self.action_lock:
            if self.active_goal_handle is not None:
                self.get_logger().warn(
                    "A detection run is already active; rejecting new goal."
                )
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Detection cancellation requested.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        with self.action_lock:
            self.active_goal_handle = goal_handle

        req = goal_handle.request
        confidence = float(req.confidence) if req.confidence > 0 else float(
            self.get_parameter("default_confidence").value
        )

        try:
            if req.input_dir:
                input_dir = Path(req.input_dir).expanduser()
                if not input_dir.exists():
                    raise FileNotFoundError(f"input_dir does not exist: {input_dir}")
            else:
                input_dir = find_latest_run(self.get_parameter("datasets_root").value)
                self.get_logger().info(f"No input_dir given; using latest run: {input_dir}")

            output_dir = (
                Path(req.output_dir).expanduser()
                if req.output_dir
                else input_dir / self.get_parameter("output_subdir").value
            )

            frames_processed, detections_found = self._run_pipeline(
                goal_handle, input_dir, output_dir, confidence
            )

            goal_handle.succeed()
            return self._make_result(
                True, output_dir, frames_processed, detections_found, ""
            )

        except GoalCancelled:
            goal_handle.canceled()
            return self._make_result(False, None, 0, 0, "Cancelled by request.")

        except Exception as exc:
            self.get_logger().error(f"Detection failed: {exc}")
            goal_handle.abort()
            return self._make_result(False, None, 0, 0, str(exc))

        finally:
            with self.action_lock:
                self.active_goal_handle = None

    def _make_result(self, success, output_dir, frames_processed, detections_found, message):
        result = DetectDents.Result()
        result.success = bool(success)
        result.output_dir = str(output_dir) if output_dir else ""
        result.frames_processed = int(frames_processed)
        result.detections_found = int(detections_found)
        result.message = message
        return result

    def _publish_feedback(self, goal_handle, stage, progress, frames_processed, total_frames):
        feedback = DetectDents.Feedback()
        feedback.stage = stage
        feedback.progress = float(progress)
        feedback.frames_processed = int(frames_processed)
        feedback.total_frames = int(total_frames)
        goal_handle.publish_feedback(feedback)

    def _check_cancel(self, goal_handle):
        if goal_handle.is_cancel_requested:
            raise GoalCancelled()

    # ------------------------------------------------------------------
    # Pipeline
    # ------------------------------------------------------------------

    def _run_pipeline(self, goal_handle, input_dir, output_dir, confidence):
        start = time.time()

        rgb_dir_in = input_dir / "rgb"
        depth_dir_in = input_dir / "depth"
        intrinsics_in = input_dir / "intrinsics.json"

        if not rgb_dir_in.is_dir():
            raise FileNotFoundError(f"No rgb/ folder in {input_dir}")

        rgb_files = sorted(rgb_dir_in.glob("*.png"))
        if not rgb_files:
            raise FileNotFoundError(f"No rgb frames found in {rgb_dir_in}")

        rgb_dir_out = output_dir / "rgb"
        depth_dir_out = output_dir / "depth"
        dent_dir_out = output_dir / "dent"
        rgb_dir_out.mkdir(parents=True, exist_ok=True)
        dent_dir_out.mkdir(parents=True, exist_ok=True)

        # depth/ and intrinsics.json are passed through unchanged.
        self._publish_feedback(goal_handle, "copying_depth_and_intrinsics", 0.0, 0, len(rgb_files))
        if depth_dir_in.is_dir():
            if depth_dir_out.exists():
                shutil.rmtree(depth_dir_out)
            shutil.copytree(depth_dir_in, depth_dir_out)
        if intrinsics_in.exists():
            shutil.copy2(intrinsics_in, output_dir / "intrinsics.json")
        else:
            self.get_logger().warn(f"No intrinsics.json found in {input_dir}; skipping copy.")

        total = len(rgb_files)
        frames_processed = 0
        detections_found = 0
        class_names = self.model.names  # {id: name}

        for idx, rgb_path in enumerate(rgb_files):
            self._check_cancel(goal_handle)
            self._publish_feedback(
                goal_handle, f"detecting_frame_{idx + 1}_of_{total}",
                idx / total, idx, total,
            )

            frame = cv2.imread(str(rgb_path))
            if frame is None:
                self.get_logger().warn(f"Failed to read {rgb_path}, skipping.")
                continue

            # rgb/ output is the frame copied through unchanged -- detected
            # or not -- so the reconstruction node sees the exact same
            # frame set/order as the original capture.
            shutil.copy2(rgb_path, rgb_dir_out / rgb_path.name)

            results = self.model.predict(
                source=frame, conf=confidence, device=self.device, verbose=False
            )
            boxes = results[0].boxes if results else None

            if boxes is not None and len(boxes) > 0:
                stem = rgb_path.stem
                for det_idx in range(len(boxes)):
                    x1, y1, x2, y2 = [float(v) for v in boxes.xyxy[det_idx].tolist()]
                    conf_score = float(boxes.conf[det_idx].item())
                    cls_id = int(boxes.cls[det_idx].item())
                    cls_name = class_names.get(cls_id, str(cls_id))

                    xi1, yi1 = max(0, int(round(x1))), max(0, int(round(y1)))
                    xi2 = min(frame.shape[1], int(round(x2)))
                    yi2 = min(frame.shape[0], int(round(y2)))
                    if xi2 <= xi1 or yi2 <= yi1:
                        self.get_logger().warn(
                            f"Degenerate bbox on {rgb_path.name} det {det_idx}, skipping."
                        )
                        continue

                    crop = frame[yi1:yi2, xi1:xi2]
                    crop_name = f"{stem}_{det_idx:03d}"
                    crop_path = dent_dir_out / f"{crop_name}.png"
                    json_path = dent_dir_out / f"{crop_name}.json"

                    if not cv2.imwrite(str(crop_path), crop):
                        self.get_logger().warn(f"Failed to write crop {crop_path}, skipping.")
                        continue

                    with open(json_path, "w") as f:
                        json.dump(
                            {
                                "frame": rgb_path.name,
                                "bbox": [xi1, yi1, xi2, yi2],
                                "confidence": conf_score,
                                "class": cls_name,
                            },
                            f, indent=2,
                        )

                    detections_found += 1

            frames_processed += 1

        self._publish_feedback(goal_handle, "done", 1.0, frames_processed, total)
        self.get_logger().info(
            f"YOLO stage done: {frames_processed}/{total} frames processed, "
            f"{detections_found} detections written to {dent_dir_out} "
            f"(took {time.time() - start:.1f}s)"
        )
        return frames_processed, detections_found


def main(args=None):
    rclpy.init(args=args)
    node = DentYoloNode()

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
