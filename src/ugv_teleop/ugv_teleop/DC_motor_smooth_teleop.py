#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import sys, select, termios, tty, threading, time

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class SmoothTeleopNode(Node):
    def __init__(self):
        super().__init__('smooth_teleop_node')
        self.pwm_pub = self.create_publisher(Int32MultiArray, 'servo_pwm_us', 10)
        self.scales_pub = self.create_publisher(Float32MultiArray, 'active_motor_scales', 10)

        # Servo calibration
        self.servo_calib = [
            {'bw_start': 1434, 'fw_start': 1548},
            {'bw_start': 1442, 'fw_start': 1554},
            {'bw_start': 1435, 'fw_start': 1548},
            {'bw_start': 1432, 'fw_start': 1544}
        ]
        self.PWM_MIN, self.PWM_MAX = 1000.0, 2000.0

        # Motion scaling presets
        self.FORWARD_SCALES      = [0.875, 1.125, 0.875, 1.125]
        self.STRAFE_RIGHT_SCALES = [1.400, 1.000, 1.265, 1.400]
        self.STRAFE_LEFT_SCALES  = [0.745, 1.000, 0.780, 0.725]
        self.ROTATE_SCALES       = [1.000, 1.000, 1.000, 1.000]
        self.STOP_SCALES         = [1.0, 1.0, 1.0, 1.0]

        # State variables
        self.servo_angles_deg = [85, 95] # [Tilt, Pan]
        self.target_speed_pct = [0.0]*4  # command speeds in %
        self.current_speed_pct = [0.0]*4
        self.last_direction = [0,0,0,0]
        self.power_level = 50.0

        # Parameters for testing
        self.ACCEL_LIMIT = 20.0       # % change per 0.05s
        self.MIN_MOVE_PWM_OFFSET = 10 # µs beyond neutral
        self.NEUTRAL_DELAY = 0.15     # seconds
        self.MAP_MODE_MAX_SPEED = 50.0 # % of full speed

        # Terminal + input thread
        self.settings = termios.tcgetattr(sys.stdin)
        self.key_thread = threading.Thread(target=self.key_loop)
        self.key_thread.daemon = True
        self.key_thread.start()

        # Update loop for ramping
        self.timer = self.create_timer(0.05, self.update_motion)  # 20 Hz

        self.get_logger().info("--- Smooth Teleop Ready ---\n"
                                "Keys: I/K=Fwd/Bwd, J/L=Strafe, U/O=Rot\n"
                                "1-9=Power %, 0=100%, Space=Stop, Q=Quit\n")

    def key_loop(self):
        tty.setraw(sys.stdin.fileno())
        self.handle_key(' ')
        while rclpy.ok():
            key = sys.stdin.read(1).lower()
            self.handle_key(key)

    def handle_key(self, key):
        fwd   = [ 1, -1,  1, -1]
        back  = [-1,  1, -1,  1]
        left  = [-1, -1,  1,  1]
        right = [ 1,  1, -1, -1]
        rleft = [-1, -1, -1, -1]
        rright= [ 1,  1,  1,  1]
        stop  = [ 0,  0,  0,  0]

        direction, scales = stop, self.STOP_SCALES
        power = self.power_level

        if key == 'i': direction, scales = fwd, self.FORWARD_SCALES
        elif key == 'k': direction, scales = back, self.FORWARD_SCALES
        elif key == 'j': direction, scales = left, self.STRAFE_LEFT_SCALES
        elif key == 'l': direction, scales = right, self.STRAFE_RIGHT_SCALES
        elif key == 'u': direction, scales = rleft, self.ROTATE_SCALES
        elif key == 'o': direction, scales = rright, self.ROTATE_SCALES
        elif key == ' ':
            # immediate stop: cut target + current speeds and publish neutral PWM
            power = 0
            self.target_speed_pct = [0.0] * 4      # stop intent
            self.current_speed_pct = [0.0] * 4     # immediate cancel of ramping
            self.last_direction = [0, 0, 0, 0]
            # publish neutral PWM immediately so motors brake to neutral
            neutral_pwms = [ (c['fw_start'] + c['bw_start']) // 2 for c in self.servo_calib ]
            self.publish_pwm(neutral_pwms)
            return  # don't continue with other motion logic
            return
        elif key == 'm': # mapping mode speed limit
            self.power_level = min(self.power_level, self.MAP_MODE_MAX_SPEED)
            self.get_logger().info(f"Mapping mode: max speed {self.power_level}%")

        if '1' <= key <= '9': self.power_level = float(key)*10.0
        elif key == '0': self.power_level = 100.0

        # Servo control
        if key in 'wsad':
            if key == 's': self.servo_angles_deg[0] = min(105, self.servo_angles_deg[0] + 5)
            elif key == 'w': self.servo_angles_deg[0] = max(0, self.servo_angles_deg[0] - 5)
            elif key == 'd': self.servo_angles_deg[1] = min(180, self.servo_angles_deg[1] + 5)
            elif key == 'a': self.servo_angles_deg[1] = max(0, self.servo_angles_deg[1] - 5)
            return  # no motion change

        # Check for direction reversal → neutral phase
        if any(d * ld < 0 for d, ld in zip(direction, self.last_direction) if ld != 0):
            self.current_speed_pct = [0.0]*4
            self.publish_pwm([ (c['fw_start']+c['bw_start'])//2 for c in self.servo_calib ])
            time.sleep(self.NEUTRAL_DELAY)

        # Set target speed for velocity profile
        self.target_speed_pct = [power * direction[i] * scales[i] for i in range(4)]
        self.last_direction = direction

        if key == 'q':
            self.shutdown()

    def update_motion(self):
        # Smoothly approach target speeds
        for i in range(4):
            diff = self.target_speed_pct[i] - self.current_speed_pct[i]
            step = self.ACCEL_LIMIT * 0.05  # per timer tick
            if abs(diff) <= step:
                self.current_speed_pct[i] = self.target_speed_pct[i]
            else:
                self.current_speed_pct[i] += step if diff > 0 else -step

        # Convert to PWM with min-move offset
        pwms = []
        for i, spd in enumerate(self.current_speed_pct):
            calib = self.servo_calib[i]
            neutral = (calib['fw_start'] + calib['bw_start']) / 2.0
            if spd > 0:
                pwm = map_value(spd, 0, 100, calib['fw_start']+self.MIN_MOVE_PWM_OFFSET, self.PWM_MAX)
            elif spd < 0:
                pwm = map_value(abs(spd), 0, 100, calib['bw_start']-self.MIN_MOVE_PWM_OFFSET, self.PWM_MIN)
            else:
                pwm = neutral
            pwms.append(int(pwm))

        self.publish_pwm(pwms)

    def publish_pwm(self, motor_pwms):
        msg = Int32MultiArray()
        msg.data = motor_pwms + self.servo_angles_deg
        self.pwm_pub.publish(msg)

        scales_msg = Float32MultiArray()
        scales_msg.data = self.target_speed_pct
        self.scales_pub.publish(scales_msg)

    def shutdown(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        self.publish_pwm([ (c['fw_start']+c['bw_start'])//2 for c in self.servo_calib ])
        self.get_logger().info("Stopping robot.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SmoothTeleopNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
