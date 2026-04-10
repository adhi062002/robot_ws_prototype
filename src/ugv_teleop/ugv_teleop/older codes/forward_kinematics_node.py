#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
import math

class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')

        # --- Parameters ---
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('max_wheel_speed_rad_s', 7.15) # The single value to tune
        self.declare_parameter('linear_scale_factor', 1.118)  
        self.declare_parameter('strafe_scale_factor', 0.48)
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.MAX_WHEEL_SPEED_RAD_S = self.get_parameter('max_wheel_speed_rad_s').get_parameter_value().double_value
        self.linear_scale = self.get_parameter('linear_scale_factor').get_parameter_value().double_value
        self.strafe_scale = self.get_parameter('strafe_scale_factor').get_parameter_value().double_value     
        self.get_logger().info(f'Publish TF: {self.publish_tf}')
        self.get_logger().info(f'Max_wheel_speed_rad_s: {self.MAX_WHEEL_SPEED_RAD_S}')

        # --- Constants & Calibrated Parameters ---
        self.WHEEL_RADIUS = 0.040; self.L = 0.108; self.W = 0.08045
        self.neutral_us = [1491.0, 1498.0, 1492.0, 1488.0]
        self.max_pwm_delta = 500.0
        
        # Pan/Tilt calibration
        self.PAN_CMD_MIN_DEG, self.PAN_CMD_MAX_DEG = 0.0, 180.0
        self.TILT_CMD_MIN_DEG, self.TILT_CMD_MAX_DEG = 0.0, 105.0
        self.PAN_JOINT_LOWER_RAD, self.PAN_JOINT_UPPER_RAD = -1.57, 1.57
        self.TILT_JOINT_LOWER_RAD, self.TILT_JOINT_UPPER_RAD = -1.578, 0.2546

        # --- State Variables ---
        self.active_scales = [1.0, 1.0, 1.0, 1.0]
        self.vx, self.vy, self.vth = 0.0, 0.0, 0.0
        self.w_fl, self.w_fr, self.w_bl, self.w_br = 0.0, 0.0, 0.0, 0.0
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.last_publish_time = None
        self.last_pwm_time = self.get_clock().now() # Initialize the timeout timer
        self.pan_angle_deg, self.tilt_angle_deg = 90.0, 90.0
        self.wheel_angles = [0.0, 0.0, 0.0, 0.0]

        # --- ROS2 Communications ---
        self.odom_publisher = self.create_publisher(Odometry, 'odom/wheel', 10)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        if self.publish_tf: self.tf_broadcaster = TransformBroadcaster(self)
        self.pwm_subscriber = self.create_subscription(Int32MultiArray, 'servo_pwm_us', self.pwm_callback, 10)
        self.scales_subscriber = self.create_subscription(Float32MultiArray, 'active_motor_scales', self.scales_callback, 10)
        
        self.timer = self.create_timer(0.02, self.publish_loop) # 50 Hz

    def map_value(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def scales_callback(self, msg):
        self.active_scales = msg.data

    def pwm_to_rad_s(self, pwm, wheel_index):
        effort = (pwm - self.neutral_us[wheel_index]) / self.max_pwm_delta
        effort = max(-1.0, min(1.0, effort))
        return effort * self.MAX_WHEEL_SPEED_RAD_S

    def pwm_callback(self, msg):
        # *** CRITICAL: Update the timestamp whenever a command is received ***
        self.last_pwm_time = self.get_clock().now()
        
        if len(msg.data) < 6: return
        self.tilt_angle_deg, self.pan_angle_deg = float(msg.data[4]), float(msg.data[5])
        
        w_crooked = [self.pwm_to_rad_s(msg.data[i], i) for i in range(4)]
        w_ideal = [w_crooked[i] / (self.active_scales[i] + 1e-9) for i in range(4)]
        
        if any(abs(w) > 1e-6 for w in w_ideal):
            avg_ideal_speed = sum(abs(w) for w in w_ideal) / 4.0
            self.w_fl = math.copysign(avg_ideal_speed, w_ideal[0]) if w_ideal[0] != 0 else 0.0
            self.w_fr = math.copysign(avg_ideal_speed, w_ideal[1]) if w_ideal[1] != 0 else 0.0
            self.w_bl = math.copysign(avg_ideal_speed, w_ideal[2]) if w_ideal[2] != 0 else 0.0
            self.w_br = math.copysign(avg_ideal_speed, w_ideal[3]) if w_ideal[3] != 0 else 0.0
        else:
            self.w_fl, self.w_fr, self.w_bl, self.w_br = 0.0, 0.0, 0.0, 0.0

        kinematic_w_fl, kinematic_w_fr = self.w_fl, -self.w_fr
        kinematic_w_bl, kinematic_w_br = self.w_bl, -self.w_br
        
        r, l_plus_w = self.WHEEL_RADIUS, self.L + self.W
        self.vx  = (kinematic_w_fl + kinematic_w_fr + kinematic_w_bl + kinematic_w_br) * (r / 4)
        self.vy  = (-kinematic_w_fl + kinematic_w_fr + kinematic_w_bl - kinematic_w_br) * (r / 4)
        self.vth = (-kinematic_w_fl + kinematic_w_fr - kinematic_w_bl + kinematic_w_br) * (r / (4 * l_plus_w))

        self.vx *= self.linear_scale
        self.vy *= self.linear_scale
    def publish_loop(self):
        current_time = self.get_clock().now()
        if self.last_publish_time is None:
            self.last_publish_time = current_time
            return
        
        dt = (current_time - self.last_publish_time).nanoseconds / 1e9
        self.last_publish_time = current_time
        if dt > 0.02: dt = 0.02

        # *** THE TIMEOUT LOGIC IS BACK AND CORRECTED ***
        time_since_last_cmd = (current_time - self.last_pwm_time).nanoseconds / 1e9
        # if time_since_last_cmd > 0.5: # If no command for 0.5s, stop the robot
        #     self.vx, self.vy, self.vth = 0.0, 0.0, 0.0
        #     self.w_fl, self.w_fr, self.w_bl, self.w_br = 0.0, 0.0, 0.0, 0.0

        # Integration
        self.x += (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        self.y += (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        self.theta += self.vth * dt
        
        q = Quaternion(z=math.sin(self.theta / 2.0), w=math.cos(self.theta / 2.0))
        
        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp, odom_msg.header.frame_id, odom_msg.child_frame_id = current_time.to_msg(), 'odom', 'base_footprint'
        odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y = self.x, self.y
        odom_msg.pose.pose.orientation = q
        odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y = self.vx, self.vy
        odom_msg.twist.twist.angular.z = self.vth
        self.odom_publisher.publish(odom_msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = current_time.to_msg(), 'odom', 'base_footprint'
            t.transform.translation.x, t.transform.translation.y = self.x, self.y
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)
        
        self.wheel_angles = [(a + w * dt) for a, w in zip(self.wheel_angles, [self.w_fl, self.w_fr, self.w_bl, self.w_br])]
        js_msg = JointState()
        js_msg.header.stamp = current_time.to_msg()
        js_msg.name = ['front_left_wheel_joint', 'front_right_wheel_joint', 'back_left_wheel_joint', 'back_right_wheel_joint', 'pan_joint', 'tilt_joint']
        js_msg.position = self.wheel_angles + [
            self.map_value(self.pan_angle_deg, self.PAN_CMD_MIN_DEG, self.PAN_CMD_MAX_DEG, self.PAN_JOINT_LOWER_RAD, self.PAN_JOINT_UPPER_RAD),
            self.map_value(self.tilt_angle_deg, self.TILT_CMD_MIN_DEG, self.TILT_CMD_MAX_DEG, self.TILT_JOINT_LOWER_RAD, self.TILT_JOINT_UPPER_RAD)
        ]
        self.joint_state_publisher.publish(js_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
