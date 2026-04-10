# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped
# from rclpy.time import Time
# import math
# import os

# class PathMeasureNode(Node):

#     def __init__(self):
#         super().__init__('path_measure_node')

#         self.state_idle = 0
#         self.state_wait_stop = 1
#         self.state_wait_path = 2

#         self.state = self.state_idle

#         self.start_time = None
#         self.stop_time = None

#         self.trigger_file = "/home/nvidia/ptrigger.txt"

#         self.create_subscription(Path, "/path", self.path_callback, 10)
#         self.create_timer(0.5, self.check_trigger_file)

#         self.get_logger().info("path_measure_node started")

#     def check_trigger_file(self):
#         if not os.path.exists(self.trigger_file):
#             return

#         try:
#             with open(self.trigger_file, 'r') as f:
#                 cmd = f.readline().strip()
#         except:
#             return

#         if cmd == "start" and self.state == self.state_idle:
#             self.start_time = self.get_clock().now()
#             self.state = self.state_wait_stop
#             self.get_logger().info("START detected. Timestamp stored.")

#         elif cmd == "stop" and self.state == self.state_wait_stop:
#             self.stop_time = self.get_clock().now()
#             self.state = self.state_wait_path
#             self.get_logger().info("STOP detected. Waiting for next Path message...")

#     def path_callback(self, msg):
#         if self.state != self.state_wait_path:
#             return

#         if not msg.poses:
#             self.get_logger().warn("Received empty Path message")
#             return

#         valid = []

#         for p in msg.poses:
#             # Construct ROS2 time correctly (Humble)
#             pose_time = Time(
#                 nanoseconds = p.header.stamp.sec * 1_000_000_000 + p.header.stamp.nanosec,
#                 clock_type = self.get_clock().clock_type
#             )

#             if self.start_time <= pose_time <= self.stop_time:
#                 valid.append(p)

#         if len(valid) < 2:
#             self.get_logger().warn("Not enough poses in time window.")
#             self.state = self.state_idle
#             return

#         # Total XY distance
#         total_dist = 0.0
#         for i in range(1, len(valid)):
#             x1 = valid[i-1].pose.position.x
#             y1 = valid[i-1].pose.position.y
#             x2 = valid[i].pose.position.x
#             y2 = valid[i].pose.position.y
#             total_dist += math.hypot(x2 - x1, y2 - y1)

#         # XY displacement (first -> last)
#         first = valid[0].pose.position
#         last = valid[-1].pose.position

#         displacement_xy = math.hypot(last.x - first.x, last.y - first.y)

#         self.get_logger().info(
#             "\n===== Path Measurement Result =====\n"
#             f"Start: {self.start_time.nanoseconds * 1e-9:.3f}s\n"
#             f"Stop:  {self.stop_time.nanoseconds * 1e-9:.3f}s\n"
#             f"Valid poses: {len(valid)}\n"
#             f"XY displacement: {displacement_xy:.6f} m\n"
#             f"Total XY distance: {total_dist:.6f} m\n"
#         )

#         self.state = self.state_idle



# def main(args=None):
#     rclpy.init(args=args)
#     node = PathMeasureNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import math
import os


class PathMeasureNode(Node):

    def __init__(self):
        super().__init__('path_measure_node')

        self.trigger_file = "/home/nvidia/ptrigger.txt"

        self.running = False
        self.total_distance = 0.0

        self.first_x = None
        self.first_y = None

        self.last_x = None
        self.last_y = None

        self.create_subscription(Path, "/path", self.path_callback, 10)
        self.create_timer(0.5, self.check_trigger_file)

        self.get_logger().info("path_measure_node started")

    def check_trigger_file(self):
        if not os.path.exists(self.trigger_file):
            return

        try:
            with open(self.trigger_file, "r") as f:
                cmd = f.readline().strip()
        except:
            return

        # START: reset state
        if cmd == "start" and not self.running:
            self.running = True
            self.total_distance = 0.0
            self.first_x = None
            self.first_y = None
            self.last_x = None
            self.last_y = None
            self.get_logger().info("START detected. Measurement reset.")

        # STOP: print result
        elif cmd == "stop" and self.running:
            self.running = False

            if self.first_x is None or self.last_x is None:
                self.get_logger().warn("STOP detected but no poses received.")
                return

            displacement = math.hypot(self.last_x - self.first_x,
                                      self.last_y - self.first_y)

            self.get_logger().info(
                f"\n===== Distance Measurement =====\n"
                f"Displacement (first→last): {displacement:.6f} m\n"
                f"Total XY distance:         {self.total_distance:.6f} m\n"
            )

    def path_callback(self, msg):
        if not self.running:
            return

        if not msg.poses:
            return

        # Use ONLY the last pose in the Path (most recent)
        pose = msg.poses[-1].pose
        x = pose.position.x
        y = pose.position.y

        # First pose after START
        if self.first_x is None:
            self.first_x = x
            self.first_y = y
            self.last_x = x
            self.last_y = y
            # First pose does not add distance
            return

        # Increment total distance
        step_dist = math.hypot(x - self.last_x, y - self.last_y)
        self.total_distance += step_dist

        # Update last position
        self.last_x = x
        self.last_y = y


def main(args=None):
    rclpy.init(args=args)
    node = PathMeasureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


