import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import struct
import threading
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import math
from sensor_msgs.msg import MagneticField
import time
from builtin_interfaces.msg import Time
def unix_to_ros_time(t):
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return Time(sec=sec, nanosec=nanosec)

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'cmd_drive',  # Topic that sends 6 microsecond values
            self.pwm_callback,
            10
        )
        
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        self.arduino = None
        self.running = False

        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.arduino.flushInput()
            self.get_logger().info("Connected to Teensy on /dev/ttyACM0")
            
            self.running = True
            self.read_thread = threading.Thread(target=self.serial_read_loop)
            self.read_thread.daemon = True
            self.read_thread.start()
            self.get_logger().info("Started serial reading thread.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Teensy: {e}")
            rclpy.shutdown()
            return
            
    def serial_read_loop(self):
        packet_format = '<c6fB'
        packet_size = struct.calcsize(packet_format)
        
        # This buffer stores bytes as they come in from serial
        read_buffer = bytearray()

        while self.running and rclpy.ok():
            if self.arduino and self.arduino.is_open:
                try:
                    # Read all available bytes from the serial buffer
                    if self.arduino.in_waiting > 0:
                        read_buffer.extend(self.arduino.read(self.arduino.in_waiting))
                    
                    # Process all complete packets in our buffer
                    while len(read_buffer) >= packet_size:
                        # Find the next start byte '$'
                        start_index = read_buffer.find(b'$')
                        
                        if start_index == -1:
                            # No start byte found, clear the buffer
                            read_buffer.clear()
                            break

                        # If a start byte is found but we don't have a full packet after it, wait for more data
                        if len(read_buffer) - start_index < packet_size:
                            # Discard the garbage before the start byte
                            read_buffer = read_buffer[start_index:]
                            break

                        # We have a potential full packet
                        full_packet = read_buffer[start_index : start_index + packet_size]
                        
                        # --- Checksum Validation ---
                        payload = full_packet[1:-1]
                        calculated_checksum = 0
                        for byte in payload:
                            calculated_checksum ^= byte
                        
                        # Unpack just the checksum from the end of the packet
                        received_checksum = struct.unpack('<B', full_packet[-1:])[0]

                        if calculated_checksum == received_checksum:
                            # Checksum is good, unpack the full packet and publish
                            unpacked_data = struct.unpack(packet_format, full_packet)
                            self.publish_imu_from_binary(unpacked_data)
                        else:
                            self.get_logger().warn("IMU checksum mismatch.", throttle_duration_sec=2)

                        # Remove the processed packet (and any garbage before it) from the buffer
                        read_buffer = read_buffer[start_index + packet_size:]

                except Exception as e:
                    self.get_logger().error(f"Error in serial read loop: {e}")

            import time
            time.sleep(0.001) 

    def publish_imu_from_binary(self, data_tuple):
        """
        Takes an unpacked tuple of IMU data and publishes it as a ROS 2 message.
        """
        try:
            # Data tuple format: (b'$', qw, qx, qy, qz, gx, gy, gz, ax, ay, az, checksum)
            ax, ay, az, gx, gy, gz = data_tuple[1:7]

            imu_msg = Imu()
            # imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.stamp = unix_to_ros_time(time.time())
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
            imu_msg.orientation_covariance[0] = -1.0

            imu_msg.linear_acceleration.x = float(ax) * 9.80665
            imu_msg.linear_acceleration.y = float(ay) * 9.80665
            imu_msg.linear_acceleration.z = float(az) * 9.80665

            imu_msg.angular_velocity.x = math.radians(float(gx))
            imu_msg.angular_velocity.y = math.radians(float(gy))
            imu_msg.angular_velocity.z = math.radians(float(gz))
            
            self.imu_publisher.publish(imu_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing raw IMU data from binary: {e}")

    def pwm_callback(self, msg):
        self.get_logger().info(f"Callback received: {msg.data}")   
        if self.arduino is not None:
            values = msg.data
            if len(values) == 2:
                left  = int(values[0])
                right = int(values[1])

                command = (

                    f"LEFT:{left} RIGHT:{right}\n "
                 #   f"servo1:{values[4]} servo2:{values[5]}\n"
                  #  f"LEFT:{left} RIGHT:{right} "
                  #  f"servo1:{values[4]} servo2:{values[5]}\n"
                )
              ##old code 
              #  command = (
              #      f"FL:{values[0]} FR:{values[1]} BL:{values[2]} BR:{values[3]} "
              #      f"servo1:{values[4]} servo2:{values[5]}\n"
              #  )
                self.arduino.write(command.encode())
                self.get_logger().info(f"Sent to Teensy: {command.strip()}")
            else:

                self.get_logger().warn("Expected 2 values (LEFT, RIGHT)")

              #  self.get_logger().warn("Expected 2 values Left & Right")

        else:
            self.get_logger().warn("Teensy not connected. Skipping command.")
            
    def destroy_node(self):
        """Called on shutdown to clean up resources."""
        self.get_logger().info("Shutting down...")
        self.running = False # Signal the thread to stop
        if hasattr(self, 'read_thread') and self.read_thread.is_alive():
            self.read_thread.join(timeout=1) # Wait for the thread to exit
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
        super().destroy_node()            

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
    	if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # The spin() function will automatically call destroy_node on shutdown.
        # This explicit call is good for clarity.
        node.destroy_node()
        rclpy.shutdown()
        
        
if __name__ == '__main__':
    main()


