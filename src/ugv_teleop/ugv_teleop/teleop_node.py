# ugv_teleop/teleop_node.py (FINAL - Publish on Keypress Version)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import sys, select, termios, tty, threading

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class CalibratedTeleopNode(Node):
    def __init__(self):
        super().__init__('calibrated_teleop_node')
        self.pwm_publisher = self.create_publisher(Int32MultiArray, 'servo_pwm_us', 10)
        self.scales_publisher = self.create_publisher(Float32MultiArray, 'active_motor_scales', 10)

        # --- COMPLETE CALIBRATION DATASET ---
        self.servo_calib = [ {'bw_start': 1434, 'fw_start': 1548}, {'bw_start': 1442, 'fw_start': 1554}, {'bw_start': 1435, 'fw_start': 1548}, {'bw_start': 1432, 'fw_start': 1544} ]
        self.PWM_MIN, self.PWM_MAX = 1000.0, 2000.0
        # self.FORWARD_SCALES      = [0.875, 1.125, 0.875, 1.125]
        # self.STRAFE_RIGHT_SCALES = [1.400, 1.000, 1.265, 1.400]
        # self.STRAFE_LEFT_SCALES  = [0.745, 1.000, 0.780, 0.725]
        # self.ROTATE_SCALES       = [1.000, 1.000, 1.000, 1.000]
        self.FORWARD_SCALES      = [0.325, 0.375, 0.325, 0.425]
        self.STRAFE_RIGHT_SCALES = [1.400, 1.000, 1.265, 1.400]
        self.STRAFE_LEFT_SCALES  = [0.745, 1.000, 0.780, 0.725]
        self.ROTATE_SCALES       = [0.200, 0.200, 0.200, 0.100]
        self.BACKWARD_SCALES     = [0.325, 0.325, 0.325, 0.325]
        self.STOP_SCALES         = [1.0, 1.0, 1.0, 1.0]

        # --- STATE VARIABLES ---
        self.servo_angles_deg = [85, 95] # [Tilt, Pan]
        self.power_level = 50.0

        # --- Terminal and Threading Setup ---
        self.settings = termios.tcgetattr(sys.stdin)
        self.key_thread = threading.Thread(target=self.key_listener_loop)
        self.key_thread.daemon = True
        self.key_thread.start()
        self.print_instructions()

    def print_instructions(self):
        self.get_logger().info("\n--- Calibrated Teleop Controller ---\n...") # (Abbreviated for clarity)
        
    def key_listener_loop(self):
        tty.setraw(sys.stdin.fileno())
        # Publish a stop command on startup
        self.handle_key_and_publish(' ')
        while rclpy.ok():
            key = sys.stdin.read(1)
            self.handle_key_and_publish(key)

    def handle_key_and_publish(self, key):
        forward      = [ 1, -1,  1, -1]; backward = [-1,  1, -1,  1]
        strafe_left  = [-1, -1,  1,  1]; strafe_right = [ 1,  1, -1, -1]
        rot_left     = [-1, -1, -1, -1]; rot_right    = [ 1,  1,  1,  1]
        stop         = [ 0,  0,  0,  0]

        key = key.lower()
        
        # Default to stop if no motion key is pressed
        current_power = self.power_level
        direction, active_scales = stop, self.STOP_SCALES
        
        # Update motion state based on key
        if key == 'i': direction, active_scales = forward, self.FORWARD_SCALES
        elif key == 'k': direction, active_scales = backward, self.BACKWARD_SCALES
        elif key == 'j': direction, active_scales = strafe_left, self.STRAFE_LEFT_SCALES
        elif key == 'l': direction, active_scales = strafe_right, self.STRAFE_RIGHT_SCALES
        elif key == 'u': direction, active_scales = rot_left, self.ROTATE_SCALES
        elif key == 'o': direction, active_scales = rot_right, self.ROTATE_SCALES
        elif key == ' ': current_power = 0
        
        # Set power level (these keys don't trigger a new motion command, just set the state for the *next* one)
        if '1' <= key <= '9': self.power_level = float(key) * 10.0
        elif key == '0': self.power_level = 100.0
        
        # Update Servo State
        if key in 'wsad':
            if key == 's': self.servo_angles_deg[0] = min(105, self.servo_angles_deg[0] + 5)
            elif key == 'w': self.servo_angles_deg[0] = max(0, self.servo_angles_deg[0] - 5)
            elif key == 'd': self.servo_angles_deg[1] = min(180, self.servo_angles_deg[1] + 5)
            elif key == 'a': self.servo_angles_deg[1] = max(0, self.servo_angles_deg[1] - 5)
            # When a servo key is pressed, we don't change the robot's motion.
            # We assume the last motion command should continue, so we don't update direction/scales.
            # We will fall through and re-publish the last known motor commands with the new servo angles.
            # This requires storing the last direction.
            # A simpler approach for now: servo keys don't republish motor commands. Let's stick to that.
            
        # --- Calculate and Publish PWM ---
        motor_pwms = [0.0] * 4
        for i in range(4):
            scaled_power = current_power * active_scales[i]
            final_power = scaled_power * direction[i]
            calib = self.servo_calib[i]
            if final_power > 0: motor_pwms[i] = map_value(final_power, 0, 100, calib['fw_start'], self.PWM_MAX)
            elif final_power < 0: motor_pwms[i] = map_value(abs(final_power), 0, 100, calib['bw_start'], self.PWM_MIN)
            else: motor_pwms[i] = (calib['fw_start'] + calib['bw_start']) / 2.0
        
        pwm_msg = Int32MultiArray()
        pwm_msg.data = [int(p) for p in motor_pwms] + self.servo_angles_deg
        self.pwm_publisher.publish(pwm_msg)
        
        scales_msg = Float32MultiArray()
        scales_msg.data = active_scales
        self.scales_publisher.publish(scales_msg)

        self.get_logger().info(f"Published Command for State: {active_scales}, PWM: {pwm_msg.data}")

        if key == 'q':
            # Send a final stop command before quitting
            stop_pwm_msg = Int32MultiArray()
            stop_pwm_msg.data = [int((c['fw_start'] + c['bw_start']) / 2) for c in self.servo_calib] + self.servo_angles_deg
            self.pwm_publisher.publish(stop_pwm_msg)
            rclpy.shutdown()

    def shutdown(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = CalibratedTeleopNode()
    rclpy.spin(node)
    # On shutdown (e.g., Ctrl+C), this part will be executed
    node.destroy_node()
