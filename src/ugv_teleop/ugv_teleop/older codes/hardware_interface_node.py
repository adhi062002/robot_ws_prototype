#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import struct
import threading
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math
import time
import time
from builtin_interfaces.msg import Time

def unix_to_ros_time(t):
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Time(sec=sec, nanosec=nanosec)

# Packet format: start byte (char), 6 floats, checksum byte
PACK_FMT = '<c6fB'   # start '$', then 6 floats, then 1-byte checksum
PACK_SIZE = struct.calcsize(PACK_FMT)
START_BYTE = b'$'    # bytes form

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher_node')

        # --- params ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('imu_frame', 'imu_link')

        # Units sent by Teensy firmware
        self.declare_parameter('accel_units', 'g')    # 'g' or 'm_s2'
        self.declare_parameter('gyro_units', 'deg_s') # 'deg_s' or 'rad_s'
        self.declare_parameter('payload_order', 'AXAYAZ_GXGYGZ')
        # If your Teensy firmware sends its own timestamp (in seconds) as part of payload
        self.declare_parameter('use_device_timestamp', False)

        # Noise densities (for covariance). Tune to sensor datasheet.
        self.declare_parameter('noise_gyro',  0.01093)  # rad/s / sqrt(Hz)
        self.declare_parameter('noise_acc', 0.000772)   # m/s^2 / sqrt(Hz)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.imu_frame = self.get_parameter('imu_frame').get_parameter_value().string_value

        self.accel_units = self.get_parameter('accel_units').get_parameter_value().string_value
        self.gyro_units = self.get_parameter('gyro_units').get_parameter_value().string_value
        self.payload_order = self.get_parameter('payload_order').get_parameter_value().string_value.upper()
        self.use_device_timestamp = bool(self.get_parameter('use_device_timestamp').get_parameter_value().bool_value)

        self.noise_g = float(self.get_parameter('noise_gyro').value)
        self.noise_a = float(self.get_parameter('noise_acc').value)

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 200)

        # Serial init
        try:
            self.arduino = serial.Serial(port, baud, timeout=0.01)
            self.arduino.reset_input_buffer()
            self.get_logger().info(f"Connected to Teensy on {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial {port}: {e}")
            self.arduino = None

        self.running = True
        self._reader = threading.Thread(target=self._serial_read_loop, daemon=True)
        self._reader.start()
        self.get_logger().info("IMU serial reader started.")

    def _serial_read_loop(self):
        buf = bytearray()
        # throttle logger for checksum warnings
        last_checksum_warn = 0.0

        while self.running and rclpy.ok():
            try:
                if self.arduino is None or not self.arduino.is_open:
                    time.sleep(0.01)
                    continue

                # read bytes available
                n = self.arduino.in_waiting
                if n:
                    buf.extend(self.arduino.read(n))

                # process packets
                while len(buf) >= PACK_SIZE:
                    # find start byte
                    idx = buf.find(START_BYTE)
                    if idx == -1:
                        # no start byte; drop buffer (nothing useful)
                        buf.clear()
                        break

                    # if start not at 0, drop earlier bytes
                    if idx > 0:
                        del buf[:idx]

                    # check if full packet is available
                    if len(buf) < PACK_SIZE:
                        break  # wait for more

                    pkt = buf[:PACK_SIZE]

                    # checksum: XOR of payload (everything except start char and last byte)
                    payload = pkt[1:-1]
                    calc = 0
                    for b in payload:
                        calc ^= b
                    rx_chk = pkt[-1]

                    if calc != rx_chk:
                        # bad packet: drop start and continue
                        now_t = time.time()
                        if now_t - last_checksum_warn > 2.0:
                            self.get_logger().warning("IMU checksum mismatch (dropping packet).")
                            last_checksum_warn = now_t
                        del buf[0:1]  # drop start byte so we can search for next
                        continue

                    # unpack
                    try:
                        unpacked = struct.unpack(PACK_FMT, pkt)
                        # unpacked: (b'$', f0..f5, chk)
                        f0, f1, f2, f3, f4, f5 = unpacked[1:7]
                    except struct.error as e:
                        self.get_logger().warning(f"IMU unpack error: {e}")
                        del buf[0:1]
                        continue

                    # map payload order into ax,ay,az,gx,gy,gz
                    if self.payload_order == 'AXAYAZ_GXGYGZ':
                        ax, ay, az, gx, gy, gz = f0, f1, f2, f3, f4, f5
                    elif self.payload_order == 'GXGYGZ_AXAYAZ':
                        gx, gy, gz, ax, ay, az = f0, f1, f2, f3, f4, f5
                    else:
                        ax, ay, az, gx, gy, gz = f0, f1, f2, f3, f4, f5

                    # units -> SI
                    if self.accel_units.lower() in ('g', 'g_force', 'g-forces'):
                        ax *= 9.80665
                        ay *= 9.80665
                        az *= 9.80665
                    # else assume m/s^2 already

                    if self.gyro_units.lower() in ('deg_s', 'deg/s', 'degrees/s'):
                        gx = math.radians(gx)
                        gy = math.radians(gy)
                        gz = math.radians(gz)
                    # else assume rad/s

                    # timestamp: optionally trust device timestamp (if firmware includes it in payload
                    # — your current packet does not include timestamp, so default to host time)
                    # stamp = self.get_clock().now().to_msg()

                    imu = Imu()
                    # imu.header.stamp = stamp
                    imu.header.frame_id = self.imu_frame
                    imu.header.stamp = unix_to_ros_time(time.time())
                    # orientation unknown => set covariance[0] = -1 (convention)
                    imu.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
                    imu.orientation_covariance[0] = -1.0

                    # angular velocity (rad/s)
                    imu.angular_velocity.x = float(gx)
                    imu.angular_velocity.y = float(gy)
                    imu.angular_velocity.z = float(gz)
                    var_g = (self.noise_g ** 2)
                    imu.angular_velocity_covariance = [0.0]*9
                    imu.angular_velocity_covariance[0] = var_g
                    imu.angular_velocity_covariance[4] = var_g
                    imu.angular_velocity_covariance[8] = var_g

                    # linear acceleration (m/s^2)
                    imu.linear_acceleration.x = float(ax)
                    imu.linear_acceleration.y = float(ay)
                    imu.linear_acceleration.z = float(az)
                    var_a = (self.noise_a ** 2)
                    imu.linear_acceleration_covariance = [0.0]*9
                    imu.linear_acceleration_covariance[0] = var_a
                    imu.linear_acceleration_covariance[4] = var_a
                    imu.linear_acceleration_covariance[8] = var_a

                    # publish
                    self.imu_pub.publish(imu)

                    # remove processed packet
                    del buf[:PACK_SIZE]

                # small sleep to yield CPU
                time.sleep(0.0005)

            except Exception as e:
                self.get_logger().error(f"Serial loop exception: {e}")
                time.sleep(0.01)

    def pwm_callback(self, msg: Int32MultiArray):
        # your existing passthrough implementation (unchanged)
        pass

    def destroy_node(self):
        self.get_logger().info("IMU node shutting down...")
        self.running = False
        if hasattr(self, '_reader') and self._reader.is_alive():
            self._reader.join(timeout=1.0)
        if self.arduino and self.arduino.is_open:
            try:
                self.arduino.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()