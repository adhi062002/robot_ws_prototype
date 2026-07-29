import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import sys, termios, tty

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')

        self.pub = self.create_publisher(Int32MultiArray, 'cmd_drive', 10)
        self.speed = 600  # max 1000

        # ✅ Positional servos (Tilt, Pan)
        self.tilt = 00
        self.pan = 90

        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print("W/S/A/D control | Arrow keys (I/K/J/L) for pan/tilt | SPACE stop | Q quit")

        while rclpy.ok():
            key = self.get_key()

            if key == 'q':
                print("Exiting teleop...")
                break

            left = 0
            right = 0

            # 🔹 Drive control
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
            # 🔹 Pan/Tilt control (separate, does NOT affect motors)
            elif key == 'i':  # tilt up
                self.tilt = max(0, self.tilt - 5)
            elif key == 'k':  # tilt down
                self.tilt = min(180, self.tilt + 5)
            elif key == 'j':  # pan left
                self.pan = max(0, self.pan - 5)
            elif key == 'l':  # pan right
                self.pan = min(180, self.pan + 5)
            #stopping    
            elif key == ' ':
                left = 0
                right = 0
            elif key == 'p':    
                self.tilt = 90
                self.pan = 90    
            else:
                continue

            msg = Int32MultiArray()

            # ✅ ALWAYS send all values
            msg.data = [left, right, self.tilt, self.pan]

            self.pub.publish(msg)

            print(f"L:{left} R:{right} | Tilt:{self.tilt} Pan:{self.pan}")


def main():
    rclpy.init()
    node = Teleop()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
