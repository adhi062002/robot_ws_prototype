import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Subscribe from teleop
        self.sub = self.create_subscription(
            Int32MultiArray,
            'cmd_drive',
            self.cmd_callback,
            10
        )

        # Publish to hardware interface
        self.pub = self.create_publisher(
            Int32MultiArray,
            'motor_cmd',
            10
        )

        # TARGET (what we want)
        self.target_left = 0
        self.target_right = 0

        # CURRENT (what robot is doing)
        self.current_left = 0
        self.current_right = 0

        # Smoothness control
        self.step = 30   # change per cycle (tune this)

        # Loop (50 Hz)
        self.timer = self.create_timer(0.02, self.update)

    def cmd_callback(self, msg):
        # Receive target from teleop
        self.target_left = msg.data[0]
        self.target_right = msg.data[1]

    def ramp(self, current, target):
        if abs(target - current) < self.step:
            return target

        if target > current:
            return current + self.step
        else:
            return current - self.step

    def update(self):
        # Smooth movement
        self.current_left = self.ramp(self.current_left, self.target_left)
        self.current_right = self.ramp(self.current_right, self.target_right)

        # Send to hardware
        msg = Int32MultiArray()
        msg.data = [self.current_left, self.current_right]
        self.pub.publish(msg)

        # Debug print (optional)
        self.get_logger().info(
            f"L:{self.current_left} R:{self.current_right}"
        )

def main():
    rclpy.init()
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
