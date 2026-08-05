#!/usr/bin/env python3
"""
capture_scan_gui_client_example.py

Minimal example of how a GUI node/process should talk to
realsense_frame_capture_node over the /capture_scan action.

This is intentionally bare-bones -- wire the two calls below
(send a goal, cancel a goal) to your actual GUI's "Start Scan" /
"Cancel" buttons.
"""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from dent_mapping_interfaces.action import CaptureScan


class CaptureScanGuiClient(Node):
    def __init__(self):
        super().__init__("capture_scan_gui_client_example")
        self._client = ActionClient(self, CaptureScan, "capture_scan")
        self._goal_handle = None

    def start_scan(self, mode: int):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("capture_scan action server not available.")
            return

        goal = CaptureScan.Goal()
        goal.mode = mode

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        send_future.add_done_callback(self._on_goal_response)

    def cancel_scan(self):
        if self._goal_handle is None:
            self.get_logger().warn("No active goal to cancel.")
            return
        self._goal_handle.cancel_goal_async()

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected (is another capture already running?).")
            return

        self._goal_handle = goal_handle
        self.get_logger().info("Goal accepted, capture running.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f"[feedback] state={fb.state} frames={fb.frames_captured}")

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(
            f"[result] success={result.success} run_dir={result.run_dir} "
            f"frames={result.frames_captured}"
        )
        self._goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = CaptureScanGuiClient()

    mode = CaptureScan.Goal.EVERY_5TH_FRAME
    if len(sys.argv) > 1 and sys.argv[1] == "manual":
        mode = CaptureScan.Goal.MANUAL

    node.start_scan(mode)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
