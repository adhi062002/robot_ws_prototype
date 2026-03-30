#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import sys, termios, tty

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')

        self.pub = self.create_publisher(Int32MultiArray, 'servo_pwm_us', 10)

        self.speed = 600  # range: -1000 to 1000
        self.servo_angles = [90, 90]

        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print("W/S/A/D → Move | SPACE → Stop | Q → Quit")
        print("I/K → Servo1 | J/L → Servo2")

        while rclpy.ok():
            key = self.get_key()

            left = 0
            right = 0

            # ===== MOTOR CONTROL =====
            if key == 'w':
                left = self.speed
                right = self.speed
            elif key == 's':
                left = -self.speed
                right = -self.speed
            elif key == 'a':
                left = -self.speed
                right = self.speed
            elif key == 'd':
                left = self.speed
                right = -self.speed
            elif key == ' ':
                left = 0
                right = 0

            # ===== SERVO CONTROL =====
            elif key == 'i':
                self.servo_angles[0] = min(180, self.servo_angles[0] + 5)
            elif key == 'k':
                self.servo_angles[0] = max(0, self.servo_angles[0] - 5)
            elif key == 'l':
                self.servo_angles[1] = min(180, self.servo_angles[1] + 5)
            elif key == 'j':
                self.servo_angles[1] = max(0, self.servo_angles[1] - 5)

            # ===== EXIT =====
            elif key == 'q':
                print("Exiting teleop...")
                break
            else:
                continue

            msg = Int32MultiArray()

            # 4 motor channels (same left/right duplicated)
            msg.data = [
                left, right,
                left, right,
                self.servo_angles[0],
                self.servo_angles[1]
            ]

            self.pub.publish(msg)

            print(f"L:{left} R:{right} | Servo:{self.servo_angles}")

        self.cleanup()

    def cleanup(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        # Send STOP before exit
        msg = Int32MultiArray()
        msg.data = [0,0,0,0, self.servo_angles[0], self.servo_angles[1]]
        self.pub.publish(msg)

        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = Teleop()
    node.run()

if __name__ == '__main__':
    main()
