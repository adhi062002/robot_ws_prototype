#!/usr/bin/env python3
"""
realsense_frame_capture_node.py

ROS 2 RealSense RGB + aligned-depth frame capture node, packaged under
dent_mapping.

Workflow:
    1. A ROS 2 Action goal (sent by the GUI) selects:
         - MANUAL capture
         - EVERY_5TH_FRAME capture
    2. A timestamped run directory is created.
    3. MANUAL mode waits for a capture key. Each key press saves one
       synchronized RGB/depth pair.
    4. EVERY_5TH_FRAME mode waits for an explicit START SCAN command.
       After START SCAN, incoming synchronized frames are counted and
       every 5th frame is saved.
    5. The Action can be cancelled at any point (by the GUI, or by the
       operator). Cancellation stops capture and finalizes the run.

Headless operation (no display, e.g. plain SSH with no X11 forwarding):
the capture_key/start_key preview-window triggers need a real display
and are unusable over a bare SSH session. Two services provide the exact
same effect without one:
    ros2 service call /trigger_capture std_srvs/srv/Trigger {}      # manual mode
    ros2 service call /trigger_start_scan std_srvs/srv/Trigger {}   # every-5th mode
Run the node with show_preview:=false to skip opening a window entirely
(it would fail/hang without a display anyway):
    ros2 run dent_mapping realsense_frame_capture_node --ros-args -p show_preview:=false
The keyboard path and the service path both work whenever a goal is
active in the matching state -- neither is exclusive of the other.

Threading model
----------------
cv2.imshow / cv2.waitKey are NOT thread-safe. The previous version called
them from inside the async execute_callback loop, which runs on the
MultiThreadedExecutor alongside the message_filters synced_callback. That
is a real race: both a capture thread and an action-executor thread could
touch OpenCV's HighGUI state concurrently.

Fix (round 2): the round-1 fix moved HighGUI calls into a ROS wall-timer
callback on their own MutuallyExclusiveCallbackGroup, which removed the
*race* but not the underlying problem -- rclpy's MultiThreadedExecutor
still runs that timer on one of its worker threads, not the process's
real main thread. cv2.imshow/waitKey are unreliable off the true main
thread on Linux (GTK/Qt backends): windows can fail to repaint and
keypresses can fail to be delivered, especially if the executor's thread
pool is busy servicing camera callbacks. That's what showed up as "stuck
preview, c/s keys don't do anything."

Real fix: the ROS2 executor now spins on a background thread (started in
main()), and the OpenCV window loop (run_gui_loop) runs on the actual
main thread as a plain while-loop -- not as any kind of ROS callback. It
reads the latest frame + state under self.data_lock, calls
imshow/waitKey, and writes state changes (capture key / start key) back
under the same lock. The action execute loops (_run_manual/
_run_every_fifth) never touch OpenCV at all -- they only read/write
self.state.

Output layout (unchanged, still consumed by dent_reconstruction_v2.py):
    <run_dir>/rgb/000000.png
    <run_dir>/depth/000000.png
    <run_dir>/intrinsics.json
"""

import json
import time
from datetime import datetime
from pathlib import Path
from threading import Event, Lock, Thread

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_srvs.srv import Trigger

from dent_mapping_interfaces.action import CaptureScan


class RealsenseFrameCaptureNode(Node):
    MANUAL = CaptureScan.Goal.MANUAL
    EVERY_5TH_FRAME = CaptureScan.Goal.EVERY_5TH_FRAME

    def __init__(self):
        super().__init__("realsense_frame_capture_node")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("output_dir", str(Path.home() / "datasets"))
        self.declare_parameter("show_preview", True)
        self.declare_parameter("sync_slop_sec", 0.03)
        self.declare_parameter("capture_key", "c")
        self.declare_parameter("start_key", "s")
        self.declare_parameter("gui_period_sec", 0.03)  # ~33 Hz

        self.output_root = Path(self.get_parameter("output_dir").value)
        self.show_preview = bool(self.get_parameter("show_preview").value)
        self.capture_key = str(self.get_parameter("capture_key").value)
        self.start_key = str(self.get_parameter("start_key").value)
        gui_period = float(self.get_parameter("gui_period_sec").value)

        self.bridge = CvBridge()

        self.latest_color = None
        self.latest_depth = None
        self.latest_stamp = None
        self.intrinsics_msg = None

        self.state = "IDLE"
        self.mode = None
        self.frame_count = 0
        self.scan_frame_count = 0

        self.run_dir = None
        self.rgb_dir = None
        self.depth_dir = None
        self.intrinsics_saved = False

        self.data_lock = Lock()
        self.action_lock = Lock()
        self.active_goal_handle = None

        # Frame I/O (sensor callbacks, action execution) can run on multiple
        # threads concurrently -- that's fine, they only touch data guarded
        # by data_lock/action_lock.
        self.io_callback_group = ReentrantCallbackGroup()

        # Set by main() when it's time for run_gui_loop() to exit.
        self.shutdown_event = Event()

        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        sync_slop = float(self.get_parameter("sync_slop_sec").value)

        self.color_sub = message_filters.Subscriber(
            self, Image, color_topic, qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=sync_slop
        )
        self.sync.registerCallback(self.synced_callback)

        self.info_sub = self.create_subscription(
            CameraInfo,
            info_topic,
            self.camera_info_callback,
            10,
            callback_group=self.io_callback_group,
        )

        self.action_server = ActionServer(
            self,
            CaptureScan,
            "capture_scan",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.io_callback_group,
        )

        # Headless-friendly equivalents of pressing capture_key/start_key in
        # the preview window -- callable over SSH with no display at all
        # (e.g. `ros2 service call /trigger_capture std_srvs/srv/Trigger {}`).
        # Both do exactly the same state check + action as their keyboard
        # counterparts in run_gui_loop, just reachable without a window.
        self.trigger_capture_srv = self.create_service(
            Trigger, "trigger_capture", self._on_trigger_capture,
            callback_group=self.io_callback_group,
        )
        self.trigger_start_srv = self.create_service(
            Trigger, "trigger_start_scan", self._on_trigger_start,
            callback_group=self.io_callback_group,
        )

        self.gui_period = gui_period

        self.get_logger().info(
            "RealSense frame capture action server ready. Action: /capture_scan "
            "(command source: GUI action client). Headless triggers available: "
            "/trigger_capture (manual mode), /trigger_start_scan (every-5th mode)."
        )

    def _on_trigger_capture(self, request, response):
        """Headless equivalent of pressing capture_key in the preview
        window -- for MANUAL mode, over SSH with no display."""
        with self.data_lock:
            if self.mode != self.MANUAL or self.state != "READY_FOR_CAPTURE":
                response.success = False
                response.message = (
                    f"Not ready for a manual capture trigger (mode={self.mode}, "
                    f"state={self.state})."
                )
                return response

            ok = self._save_current_frame_locked()
            response.success = bool(ok)
            response.message = "Captured." if ok else "Capture failed -- see node log."
            return response

    def _on_trigger_start(self, request, response):
        """Headless equivalent of pressing start_key in the preview window
        -- for EVERY_5TH_FRAME mode, over SSH with no display."""
        with self.data_lock:
            if self.mode != self.EVERY_5TH_FRAME or self.state != "WAITING_FOR_START":
                response.success = False
                response.message = (
                    f"Not waiting to start a scan (mode={self.mode}, "
                    f"state={self.state})."
                )
                return response

            self.scan_frame_count = 0
            self.state = "CAPTURING"
            self.get_logger().info(
                "START SCAN received via /trigger_start_scan. Counting frames "
                "now; every 5th frame will be saved."
            )
            response.success = True
            response.message = "Scan started."
            return response

    # ------------------------------------------------------------------
    # ROS 2 Action handling
    # ------------------------------------------------------------------

    def goal_callback(self, goal_request):
        with self.action_lock:
            if self.active_goal_handle is not None:
                self.get_logger().warn(
                    "A capture action is already active; rejecting new goal."
                )
                return GoalResponse.REJECT

            if goal_request.mode not in (self.MANUAL, self.EVERY_5TH_FRAME):
                self.get_logger().error(f"Invalid capture mode: {goal_request.mode}")
                return GoalResponse.REJECT

            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Capture cancellation requested.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        with self.action_lock:
            self.active_goal_handle = goal_handle

        self.mode = int(goal_handle.request.mode)
        self._create_run()

        try:
            if self.mode == self.MANUAL:
                result = self._run_manual(goal_handle)
            else:
                result = self._run_every_fifth(goal_handle)
            return result
        finally:
            self._finalize_run()
            with self.action_lock:
                self.active_goal_handle = None
            self.mode = None
            self.state = "IDLE"

    def _run_manual(self, goal_handle):
        with self.data_lock:
            self.state = "READY_FOR_CAPTURE"
        self._publish_feedback(goal_handle)

        self.get_logger().info(
            f"Manual capture ready. Press '{self.capture_key}' in the preview "
            "window to capture a frame, or cancel the Action to exit."
        )

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._make_result(True)

            self._publish_feedback(goal_handle)
            time.sleep(0.05)

        goal_handle.abort()
        return self._make_result(False)

    def _run_every_fifth(self, goal_handle):
        with self.data_lock:
            self.state = "WAITING_FOR_START"
            self.scan_frame_count = 0
        self._publish_feedback(goal_handle)

        self.get_logger().info(
            f"Every-5th-frame mode ready. Position the robot, then press "
            f"'{self.start_key}' in the preview window to START SCAN, or "
            "cancel the Action to exit."
        )

        while rclpy.ok():
            with self.data_lock:
                still_waiting = self.state == "WAITING_FOR_START"
            if not still_waiting:
                break
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._make_result(True)
            self._publish_feedback(goal_handle)
            time.sleep(0.05)

        if not rclpy.ok():
            goal_handle.abort()
            return self._make_result(False)

        with self.data_lock:
            started = self.state == "CAPTURING"
        if not started:
            goal_handle.abort()
            return self._make_result(False)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._make_result(True)
            self._publish_feedback(goal_handle)
            time.sleep(0.05)

        goal_handle.abort()
        return self._make_result(False)

    def _make_result(self, success):
        result = CaptureScan.Result()
        result.success = bool(success)
        result.run_dir = str(self.run_dir) if self.run_dir else ""
        result.frames_captured = int(self.frame_count)
        return result

    def _publish_feedback(self, goal_handle):
        feedback = CaptureScan.Feedback()
        feedback.frames_captured = int(self.frame_count)
        feedback.state = str(self.state)
        goal_handle.publish_feedback(feedback)

    # ------------------------------------------------------------------
    # Camera input (runs on io_callback_group threads)
    # ------------------------------------------------------------------

    def camera_info_callback(self, msg: CameraInfo):
        with self.data_lock:
            self.intrinsics_msg = msg
            if self.run_dir is None or self.intrinsics_saved:
                return
            self._save_intrinsics(msg)

    def synced_callback(self, color_msg: Image, depth_msg: Image):
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert synchronized frame: {exc}")
            return

        with self.data_lock:
            self.latest_color = color.copy()
            self.latest_depth = depth.copy()
            self.latest_stamp = color_msg.header.stamp

            if (
                self.run_dir is not None
                and self.intrinsics_msg is not None
                and not self.intrinsics_saved
            ):
                self._save_intrinsics(self.intrinsics_msg)

            if self.mode == self.EVERY_5TH_FRAME and self.state == "CAPTURING":
                self.scan_frame_count += 1
                if self.scan_frame_count % 5 == 0:
                    self._save_current_frame_locked()

    # ------------------------------------------------------------------
    # GUI / keyboard -- the ONLY place that touches cv2 HighGUI.
    # Called from main() on the process's real main thread, as a plain
    # while-loop -- NOT from any ROS executor thread/timer/callback.
    # ------------------------------------------------------------------

    def run_gui_loop(self):
        if not self.show_preview:
            return

        period_ms = max(1, int(self.gui_period * 1000))

        while rclpy.ok() and not self.shutdown_event.is_set():
            with self.data_lock:
                color = self.latest_color
                depth = self.latest_depth
                state = self.state
                mode = self.mode

            if color is not None and depth is not None:
                self._show_preview(color, depth, state)

            key = cv2.waitKey(period_ms) & 0xFF
            if key == 255:
                continue

            try:
                key_char = chr(key).lower()
            except ValueError:
                continue

            with self.data_lock:
                if mode == self.MANUAL and self.state == "READY_FOR_CAPTURE":
                    if key_char == self.capture_key.lower():
                        self._save_current_frame_locked()
                elif (
                    mode == self.EVERY_5TH_FRAME
                    and self.state == "WAITING_FOR_START"
                ):
                    if key_char == self.start_key.lower():
                        self.scan_frame_count = 0
                        self.state = "CAPTURING"
                        self.get_logger().info(
                            "START SCAN received. Counting frames now; "
                            "every 5th frame will be saved."
                        )

        cv2.destroyAllWindows()

    def _show_preview(self, color, depth, state):
        try:
            depth_vis = cv2.convertScaleAbs(depth, alpha=0.03)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            preview = np.hstack([color, depth_vis])

            if state == "WAITING_FOR_START":
                text = "WAITING: press START key"
            elif state == "CAPTURING":
                text = "SCANNING: every 5th frame"
            elif state == "READY_FOR_CAPTURE":
                text = "READY: press CAPTURE key"
            else:
                text = state

            cv2.putText(
                preview, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (255, 255, 255), 2, cv2.LINE_AA,
            )
            cv2.imshow("RealSense preview (color | depth)", preview)
        except Exception as exc:
            self.get_logger().warn(f"Preview error: {exc}")

    # ------------------------------------------------------------------
    # Run/dataset management
    # ------------------------------------------------------------------

    def _create_run(self):
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        run_dir = self.output_root / timestamp

        suffix = 1
        while run_dir.exists():
            run_dir = self.output_root / f"{timestamp}_{suffix:02d}"
            suffix += 1

        self.run_dir = run_dir
        self.rgb_dir = run_dir / "rgb"
        self.depth_dir = run_dir / "depth"
        self.rgb_dir.mkdir(parents=True, exist_ok=False)
        self.depth_dir.mkdir(parents=True, exist_ok=False)

        self.frame_count = 0
        self.scan_frame_count = 0
        self.intrinsics_saved = False

        with self.data_lock:
            if self.intrinsics_msg is not None:
                self._save_intrinsics(self.intrinsics_msg)

        self.get_logger().info(f"Created capture run: {self.run_dir}")

    def _save_intrinsics(self, msg: CameraInfo):
        if self.run_dir is None:
            return

        fx, fy = msg.k[0], msg.k[4]
        cx, cy = msg.k[2], msg.k[5]
        intr_path = self.run_dir / "intrinsics.json"

        with open(intr_path, "w") as f:
            json.dump(
                {"fx": fx, "fy": fy, "cx": cx, "cy": cy,
                 "width": msg.width, "height": msg.height},
                f, indent=2,
            )

        self.intrinsics_saved = True
        self.get_logger().info(
            f"Saved intrinsics: fx={fx:.3f} fy={fy:.3f} cx={cx:.3f} cy={cy:.3f}"
        )

    def _save_current_frame_locked(self):
        """Caller must already hold self.data_lock."""
        if self.latest_color is None or self.latest_depth is None:
            self.get_logger().warn(
                "Capture requested but no synchronized frame is available."
            )
            return False

        if self.rgb_dir is None or self.depth_dir is None:
            self.get_logger().error("Capture requested before a run directory was created.")
            return False

        idx_str = f"{self.frame_count:06d}"
        rgb_path = self.rgb_dir / f"{idx_str}.png"
        depth_path = self.depth_dir / f"{idx_str}.png"

        rgb_ok = cv2.imwrite(str(rgb_path), self.latest_color)
        depth_ok = cv2.imwrite(str(depth_path), self.latest_depth.astype(np.uint16))

        if not rgb_ok or not depth_ok:
            if rgb_ok:
                rgb_path.unlink(missing_ok=True)
            if depth_ok:
                depth_path.unlink(missing_ok=True)
            self.get_logger().error(f"Failed to save frame {idx_str}.")
            return False

        self.frame_count += 1
        self.get_logger().info(f"Saved frame {idx_str} -> {rgb_path.name}, {depth_path.name}")
        return True

    def _finalize_run(self):
        if self.run_dir is None:
            return
        self.get_logger().info(
            f"Capture run finalized: {self.run_dir} ({self.frame_count} frames)"
        )
        self.run_dir = None
        self.rgb_dir = None
        self.depth_dir = None
        self.frame_count = 0
        self.scan_frame_count = 0

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseFrameCaptureNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    # ROS callbacks (action execution, camera subscriptions) run on this
    # background thread. cv2 HighGUI must NOT be touched from here.
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # OpenCV windows/keys only work reliably on the real main thread --
        # run the preview/keyboard loop here, not inside any ROS callback.
        node.run_gui_loop()
        # If show_preview is False, run_gui_loop returns immediately;
        # block here instead so the process stays up.
        while rclpy.ok() and not node.shutdown_event.is_set():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_event.set()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
