import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import sys, termios, tty

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')

        self.pub = self.create_publisher(Int32MultiArray, 'cmd_drive', 10)
        self.speed = 600  # max 1000

        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print("W/S/A/D control | SPACE stop | Q to quit")

        while rclpy.ok():
            key = self.get_key()

            # 🔴 EXIT CONDITION
            if key == 'q':
                print("Exiting teleop...")
                break

            left = 0
            right = 0

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
            else:
                continue

            msg = Int32MultiArray()
            msg.data = [left, right]
            self.pub.publish(msg)

            print(f"LEFT:{left} RIGHT:{right}")


def main():
    rclpy.init()
    node = Teleop()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # ✅ Restore terminal properly
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
