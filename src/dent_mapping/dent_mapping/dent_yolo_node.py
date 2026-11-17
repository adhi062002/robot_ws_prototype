#!/usr/bin/env python3
"""
dent_yolo_node.py

ROS 2 YOLO-NAS dent detection node using a TensorRT 10.x engine.

Input:
  <capture_dir>/rgb/*.png
  <capture_dir>/depth/*
  <capture_dir>/intrinsics.json

Output:
  <output_dir>/rgb/<frame>.png
  <output_dir>/depth/<frame>
  <output_dir>/intrinsics.json
  <output_dir>/dent/<frame_stem>_crop.png
  <output_dir>/dent/<frame_stem>_crop.json

One highest-confidence dent crop is saved per source frame so the crop name
maps directly back to its source frame for the reconstruction stage.

ONNX export contract:
  * SuperGradients YOLO-NAS export API.
  * uint8 NCHW input, batch 1, 3 channels, normally 640x640.
  * Preprocessing and NMS included in the ONNX graph.
  * Preferred output: BATCH_FORMAT, four fixed-shape outputs in this order:
      num_predictions [B,1], pred_boxes [B,N,4],
      pred_scores [B,N], pred_classes [B,N].
  * A fixed-shape FLAT_FORMAT [N,7] output is also accepted. A flat output
    with data-dependent/dynamic N is rejected; use BATCH_FORMAT instead.

The confidence threshold used during ONNX export is baked into the graph.
The action confidence value can filter returned detections further, but
cannot recover detections already removed during export. Export with a low
threshold (e.g. 0.001) if the action should control the effective threshold.

The checkpoint's image processor uses longest-side resize and bottom/right
padding with value 114. This node applies that geometry and passes uint8 BGR
NCHW to the exported graph. Channel conversion/normalization are expected to
be embedded in the SuperGradients export graph.
"""

import json
import shutil
import time
from pathlib import Path
from threading import Lock

import cv2
import numpy as np
import torch
import tensorrt as trt

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from dent_mapping_interfaces.action import DetectDents


class GoalCancelled(Exception):
    """Raised internally to unwind out of the pipeline on cancellation."""


def find_latest_run(datasets_root):
    """Return the latest capture directory containing rgb/ and depth/."""
    datasets_root = Path(datasets_root).expanduser()
    if not datasets_root.exists():
        raise FileNotFoundError(f"Datasets root does not exist: {datasets_root}")

    candidates = [
        p for p in datasets_root.iterdir()
        if p.is_dir() and (p / "rgb").is_dir() and (p / "depth").is_dir()
    ]
    if not candidates:
        raise FileNotFoundError(
            f"No capture runs with rgb/ and depth/ found under {datasets_root}"
        )
    return sorted(candidates, key=lambda p: p.name)[-1]


class TensorRTYoloNas:
    """TensorRT 10.x runner using PyTorch CUDA tensors as device buffers."""

    def __init__(self, engine_path, logger):
        self.logger = logger
        self.engine_path = Path(engine_path).expanduser()

        if not self.engine_path.is_file():
            raise FileNotFoundError(f"TensorRT engine not found: {self.engine_path}")
        if not torch.cuda.is_available():
            raise RuntimeError(
                "CUDA is not available through PyTorch. TensorRT inference "
                "requires a working CUDA-enabled PyTorch installation on this Jetson."
            )

        self.trt_logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.trt_logger)

        self.logger.info(f"Deserializing TensorRT engine: {self.engine_path}")
        with self.engine_path.open("rb") as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())

        if self.engine is None:
            raise RuntimeError(
                "TensorRT failed to deserialize the engine. Rebuild it on this "
                "Jetson with its installed TensorRT version."
            )

        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError("TensorRT could not create an execution context.")

        self.input_names = []
        self.output_names = []
        for index in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(index)
            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
            else:
                self.output_names.append(name)

        if len(self.input_names) != 1:
            raise RuntimeError(
                f"Expected exactly one TensorRT input tensor; found {self.input_names}"
            )
        if len(self.output_names) not in (1, 4):
            raise RuntimeError(
                "Expected one output (fixed FLAT_FORMAT) or four outputs "
                f"(BATCH_FORMAT); found {self.output_names}"
            )

        self.input_name = self.input_names[0]
        input_shape = tuple(self.engine.get_tensor_shape(self.input_name))
        if len(input_shape) != 4:
            raise RuntimeError(f"Expected NCHW input [1,3,H,W], got {input_shape}")
        if input_shape[0] not in (1, -1) or input_shape[1] not in (3, -1):
            raise RuntimeError(f"Expected batch 1 and 3 input channels, got {input_shape}")

        self.input_height = int(input_shape[2]) if input_shape[2] > 0 else 640
        self.input_width = int(input_shape[3]) if input_shape[3] > 0 else 640
        if self.input_height <= 0 or self.input_width <= 0:
            raise RuntimeError(f"Invalid engine input shape: {input_shape}")

        input_dtype = self.engine.get_tensor_dtype(self.input_name)
        if input_dtype != trt.uint8:
            raise RuntimeError(
                f"Expected uint8 ONNX input (SuperGradients export default), "
                f"but TensorRT input dtype is {input_dtype}. Check the ONNX export."
            )

        if -1 in input_shape:
            wanted_shape = (1, 3, self.input_height, self.input_width)
            if not self.context.set_input_shape(self.input_name, wanted_shape):
                raise RuntimeError(f"Could not set TensorRT input shape to {wanted_shape}")

        self.output_shapes = {}
        for name in self.output_names:
            shape = tuple(self.context.get_tensor_shape(name))
            if any(dim < 0 for dim in shape):
                raise RuntimeError(
                    f"TensorRT output '{name}' has dynamic/data-dependent shape {shape}. "
                    "Export YOLO-NAS with DetectionOutputFormatMode.BATCH_FORMAT "
                    "(four fixed-shape outputs), then rebuild the engine."
                )
            self.output_shapes[name] = shape

        if len(self.output_names) == 4:
            self._validate_batch_outputs()
        else:
            shape = self.output_shapes[self.output_names[0]]
            if len(shape) not in (2, 3) or shape[-1] != 7:
                raise RuntimeError(
                    f"Single output must be fixed FLAT_FORMAT [N,7] or [1,N,7], got {shape}"
                )

        self.logger.info(
            "TensorRT engine ready: input=%s %s, outputs=%s"
            % (self.input_name, input_shape, self.output_shapes)
        )

    def _validate_batch_outputs(self):
        # SuperGradients BATCH_FORMAT output order:
        # num_predictions, pred_boxes, pred_scores, pred_classes.
        shapes = [self.output_shapes[name] for name in self.output_names]
        count_shape, boxes_shape, scores_shape, classes_shape = shapes

        if len(count_shape) != 2 or count_shape[-1] != 1:
            raise RuntimeError(
                f"Expected first BATCH_FORMAT output [B,1], got {count_shape}"
            )
        if len(boxes_shape) != 3 or boxes_shape[-1] != 4:
            raise RuntimeError(
                f"Expected second BATCH_FORMAT output [B,N,4], got {boxes_shape}"
            )
        if len(scores_shape) != 2 or len(classes_shape) != 2:
            raise RuntimeError(
                f"Expected scores/classes [B,N], got {scores_shape}, {classes_shape}"
            )
        if scores_shape != classes_shape or boxes_shape[:2] != scores_shape:
            raise RuntimeError(f"Inconsistent BATCH_FORMAT output shapes: {shapes}")

    @staticmethod
    def _torch_dtype(tensor_dtype):
        mapping = {
            trt.float32: torch.float32,
            trt.float16: torch.float16,
            trt.int32: torch.int32,
            trt.int8: torch.int8,
            trt.uint8: torch.uint8,
            trt.bool: torch.bool,
        }
        if hasattr(trt, "int64"):
            mapping[trt.int64] = torch.int64
        if tensor_dtype not in mapping:
            raise RuntimeError(f"Unsupported TensorRT output dtype: {tensor_dtype}")
        return mapping[tensor_dtype]

    def infer(self, image_bgr):
        """Infer one BGR frame; return boxes in original image coordinates."""
        orig_h, orig_w = image_bgr.shape[:2]
        scale = min(
            self.input_width / float(orig_w),
            self.input_height / float(orig_h),
        )
        resized_w = max(1, min(self.input_width, int(round(orig_w * scale))))
        resized_h = max(1, min(self.input_height, int(round(orig_h * scale))))

        resized = cv2.resize(
            image_bgr, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR
        )
        padded = cv2.copyMakeBorder(
            resized,
            top=0,
            bottom=self.input_height - resized_h,
            left=0,
            right=self.input_width - resized_w,
            borderType=cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )

        # Input is uint8 NCHW. Exported graph is expected to contain the
        # checkpoint's channel conversion and normalization.
        input_np = np.ascontiguousarray(padded.transpose(2, 0, 1)[None, ...])
        input_tensor = torch.from_numpy(input_np).to("cuda")

        if not self.context.set_tensor_address(
            self.input_name, int(input_tensor.data_ptr())
        ):
            raise RuntimeError(f"Could not bind TensorRT input {self.input_name}")

        output_tensors = {}
        for name in self.output_names:
            shape = self.output_shapes[name]
            dtype = self._torch_dtype(self.engine.get_tensor_dtype(name))
            tensor = torch.empty(shape, dtype=dtype, device="cuda")
            if not self.context.set_tensor_address(name, int(tensor.data_ptr())):
                raise RuntimeError(f"Could not bind TensorRT output {name}")
            output_tensors[name] = tensor

        stream = torch.cuda.current_stream()
        if not self.context.execute_async_v3(stream_handle=stream.cuda_stream):
            raise RuntimeError("TensorRT execute_async_v3 returned failure.")
        stream.synchronize()

        outputs = {
            name: tensor.detach().cpu().numpy()
            for name, tensor in output_tensors.items()
        }
        if len(self.output_names) == 4:
            detections = self._decode_batch_outputs(outputs)
        else:
            detections = self._decode_flat_output(outputs[self.output_names[0]])

        # Checkpoint processing uses bottom/right padding, so there is no
        # top/left padding offset to remove.
        scale_x = resized_w / float(orig_w)
        scale_y = resized_h / float(orig_h)
        mapped = []
        for x1, y1, x2, y2, score, class_id in detections:
            x1 = max(0.0, min(float(orig_w), x1 / scale_x))
            x2 = max(0.0, min(float(orig_w), x2 / scale_x))
            y1 = max(0.0, min(float(orig_h), y1 / scale_y))
            y2 = max(0.0, min(float(orig_h), y2 / scale_y))
            if x2 > x1 and y2 > y1:
                mapped.append((x1, y1, x2, y2, score, class_id))
        return mapped

    def _decode_batch_outputs(self, outputs):
        num_name, boxes_name, scores_name, classes_name = self.output_names
        count = int(np.asarray(outputs[num_name]).reshape(-1)[0])
        boxes = np.asarray(outputs[boxes_name])[0]
        scores = np.asarray(outputs[scores_name])[0]
        classes = np.asarray(outputs[classes_name])[0]

        count = max(0, min(count, len(boxes), len(scores), len(classes)))
        return [
            (
                float(boxes[i][0]), float(boxes[i][1]),
                float(boxes[i][2]), float(boxes[i][3]),
                float(scores[i]), int(classes[i]),
            )
            for i in range(count)
        ]

    @staticmethod
    def _decode_flat_output(output):
        flat = np.asarray(output)
        if flat.ndim == 3:
            if flat.shape[0] != 1:
                raise RuntimeError(f"Only batch size 1 is supported, got {flat.shape}")
            flat = flat[0]
        if flat.ndim != 2 or flat.shape[1] != 7:
            raise RuntimeError(f"Unexpected flat detection output shape: {flat.shape}")

        detections = []
        for row in flat:
            batch_index, x1, y1, x2, y2, score, class_id = [float(v) for v in row]
            if int(batch_index) == 0:
                detections.append(
                    (x1, y1, x2, y2, score, int(class_id))
                )
        return detections


class DentYoloNode(Node):
    def __init__(self):
        super().__init__("dent_yolo_node")

        default_engine = str(
            Path.home() / "robot_ws" / "models" / "dent_yolo_nas_s_fp16.engine"
        )
        self.declare_parameter("engine_path", default_engine)
        self.declare_parameter("datasets_root", str(Path.home() / "datasets"))
        self.declare_parameter("output_subdir", "yolo_output")
        self.declare_parameter("default_confidence", 0.25)
        self.declare_parameter("dent_class_name", "dent")

        engine_path = self.get_parameter("engine_path").value
        if not engine_path:
            raise RuntimeError(
                "The 'engine_path' parameter is required. Set it to the TensorRT .engine file."
            )

        self.action_lock = Lock()
        self.goal_reserved = False
        self.io_callback_group = ReentrantCallbackGroup()

        self.detector = TensorRTYoloNas(engine_path, self.get_logger())
        self.class_names = {0: self.get_parameter("dent_class_name").value}

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
            "Dent YOLO-NAS TensorRT action server ready. Action: /detect_dents"
        )

    def goal_callback(self, goal_request):
        with self.action_lock:
            if self.goal_reserved:
                self.get_logger().warn(
                    "A detection run is already active; rejecting new goal."
                )
                return GoalResponse.REJECT
            self.goal_reserved = True
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Detection cancellation requested.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        confidence = float(req.confidence) if req.confidence > 0 else float(
            self.get_parameter("default_confidence").value
        )

        try:
            if req.input_dir:
                input_dir = Path(req.input_dir).expanduser()
                if not input_dir.is_dir():
                    raise FileNotFoundError(
                        f"input_dir does not exist or is not a directory: {input_dir}"
                    )
            else:
                input_dir = find_latest_run(
                    self.get_parameter("datasets_root").value
                )
                self.get_logger().info(
                    f"No input_dir given; using latest run: {input_dir}"
                )

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
            return self._make_result(
                False, None, 0, 0, "Cancelled by request."
            )
        except Exception as exc:
            self.get_logger().error(f"Detection failed: {exc}")
            goal_handle.abort()
            return self._make_result(False, None, 0, 0, str(exc))
        finally:
            with self.action_lock:
                self.goal_reserved = False

    def _make_result(
        self, success, output_dir, frames_processed, detections_found, message
    ):
        result = DetectDents.Result()
        result.success = bool(success)
        result.output_dir = str(output_dir) if output_dir else ""
        result.frames_processed = int(frames_processed)
        result.detections_found = int(detections_found)
        result.message = message
        return result

    def _publish_feedback(
        self, goal_handle, stage, progress, frames_processed, total_frames
    ):
        feedback = DetectDents.Feedback()
        feedback.stage = stage
        feedback.progress = float(progress)
        feedback.frames_processed = int(frames_processed)
        feedback.total_frames = int(total_frames)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _check_cancel(goal_handle):
        if goal_handle.is_cancel_requested:
            raise GoalCancelled()

    def _run_pipeline(self, goal_handle, input_dir, output_dir, confidence):
        start = time.time()
        input_dir = input_dir.resolve()
        output_dir = output_dir.resolve()

        if output_dir == input_dir:
            raise ValueError(
                "output_dir must differ from input_dir; refusing to overwrite capture data."
            )

        rgb_dir_in = input_dir / "rgb"
        depth_dir_in = input_dir / "depth"
        intrinsics_in = input_dir / "intrinsics.json"

        if not rgb_dir_in.is_dir():
            raise FileNotFoundError(f"No rgb/ folder in {input_dir}")
        if not depth_dir_in.is_dir():
            raise FileNotFoundError(f"No depth/ folder in {input_dir}")
        if not intrinsics_in.is_file():
            raise FileNotFoundError(f"No intrinsics.json found in {input_dir}")

        rgb_files = sorted(rgb_dir_in.glob("*.png"))
        if not rgb_files:
            raise FileNotFoundError(f"No RGB PNG frames found in {rgb_dir_in}")

        # A new scan gets a clean, self-contained output directory.
        if output_dir.exists():
            shutil.rmtree(output_dir)

        rgb_dir_out = output_dir / "rgb"
        depth_dir_out = output_dir / "depth"
        dent_dir_out = output_dir / "dent"
        rgb_dir_out.mkdir(parents=True, exist_ok=True)
        dent_dir_out.mkdir(parents=True, exist_ok=True)

        self._publish_feedback(
            goal_handle, "copying_depth_and_intrinsics", 0.0, 0, len(rgb_files)
        )
        shutil.copytree(depth_dir_in, depth_dir_out)
        shutil.copy2(intrinsics_in, output_dir / "intrinsics.json")

        total = len(rgb_files)
        frames_processed = 0
        detections_found = 0
        self.get_logger().info(
            f"Starting TensorRT inference: {total} frames, "
            f"runtime confidence threshold={confidence:.3f}"
        )

        for idx, rgb_path in enumerate(rgb_files):
            self._check_cancel(goal_handle)
            self._publish_feedback(
                goal_handle,
                f"detecting_frame_{idx + 1}_of_{total}",
                idx / float(total),
                idx,
                total,
            )

            frame = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn(
                    f"Failed to read {rgb_path}, skipping this frame."
                )
                continue

            # Preserve all RGB frames, including frames with no detections.
            shutil.copy2(rgb_path, rgb_dir_out / rgb_path.name)

            detections = self.detector.infer(frame)
            valid = [
                det for det in detections
                if det[4] >= confidence and det[5] == 0
            ]

            # Reconstruction maps <stem>_crop.png to rgb/<stem>.png and
            # depth/<stem>.png. Keep the highest-confidence dent per frame.
            if valid:
                x1, y1, x2, y2, score, class_id = max(
                    valid, key=lambda det: det[4]
                )
                xi1 = max(0, int(round(x1)))
                yi1 = max(0, int(round(y1)))
                xi2 = min(frame.shape[1], int(round(x2)))
                yi2 = min(frame.shape[0], int(round(y2)))

                if xi2 > xi1 and yi2 > yi1:
                    crop = frame[yi1:yi2, xi1:xi2]
                    crop_stem = f"{rgb_path.stem}_crop"
                    crop_path = dent_dir_out / f"{crop_stem}.png"
                    json_path = dent_dir_out / f"{crop_stem}.json"

                    if cv2.imwrite(str(crop_path), crop):
                        with json_path.open("w") as f:
                            json.dump(
                                {
                                    "frame": rgb_path.name,
                                    "bbox": [xi1, yi1, xi2, yi2],
                                    "confidence": float(score),
                                    "class": self.class_names.get(
                                        int(class_id), str(class_id)
                                    ),
                                },
                                f,
                                indent=2,
                            )
                        detections_found += 1
                    else:
                        self.get_logger().warn(f"Failed to write crop {crop_path}")
                else:
                    self.get_logger().warn(
                        f"Degenerate bbox for {rgb_path.name}; crop skipped."
                    )

            frames_processed += 1

        self._publish_feedback(goal_handle, "done", 1.0, frames_processed, total)
        self.get_logger().info(
            f"YOLO-NAS TensorRT stage done: {frames_processed}/{total} frames "
            f"processed, {detections_found} dent crops written to {dent_dir_out} "
            f"(took {time.time() - start:.1f}s)"
        )
        return frames_processed, detections_found


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = DentYoloNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
