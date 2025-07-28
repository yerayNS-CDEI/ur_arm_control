import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Header
import serial
import re

class VL6180MultiNode(Node):
    def __init__(self):
        super().__init__('sensors_arduino')

        self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.pub_s4 = self.create_publisher(Range, 'vl6180/sensor1', 10)
        self.pub_s5 = self.create_publisher(Range, 'vl6180/sensor2', 10)
        self.pub_s6 = self.create_publisher(Range, 'vl6180/sensor3', 10)
        self.pub_s1 = self.create_publisher(Range, 'hcsr04/sensor1', 10)
        self.pub_s2 = self.create_publisher(Range, 'hcsr04/sensor2', 10)
        self.pub_s3 = self.create_publisher(Range, 'hcsro4/sensor3', 10)
        self.pub_sensors = self.create_publisher(Range, 'distance_sensors', 10)

        self.timer = self.create_timer(0.1, self.read_serial)

        self.get_logger().info("VL6180 multi-sensor node initialized.")

    def create_range_msg(self, frame_id, distance_m):
        msg = Range()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.0349
        msg.min_range = 0.01
        msg.max_range = 0.18
        msg.range = min(max(distance_m, msg.min_range), msg.max_range)
        return msg

    def read_serial(self):
        try:
            while self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()

            match = re.match(r'Ultra_dist_1:(\d+)\s+Ultra_dist_2:(\d+)\s+Ultra_dist_3:(\d+)\s+S1:(\d+)\s+S2:(\d+)\s+S3:(\d+)', line)
            if match:
                du1, du2, du3, d1, d2, d3 = map(int, match.groups())
                if all(dist != 255 for dist in (d1, d2, d3)) or all(dist < 400 for dist in (du1, du2, du3)):
                    for i, dist_mm in enumerate([d1, d2, d3], start=1):
                        # if dist_mm == 255:      # 255 is the value asigned for errors in sensor readings
                        #     continue  # ignored
                        dist_m = dist_mm / 1000.0
                        msg = self.create_range_msg(f"vl6180_sensor{i}", dist_m)
                        getattr(self, f'pub_s{i+3}').publish(msg)
                        self.get_logger().info(f"Sensor {i}: {msg.range:.3f} m")
                    for i, dist_cm in enumerate([du1, du2, du3], start=1):
                        dist_m = dist_cm / 100.0
                        msg = self.create_range_msg(f"_sensor{i}", dist_m)
                        getattr(self, f'pub_s{i}').publish(msg)
                        self.get_logger().info(f"Sensor {i}: {msg.range:.3f} m")
                
                else:
                    self.get_logger().warn("One or more invalid readings (255 or 794). Not publishing.")
                    invalid = [i+1 for i, d in enumerate([du1, du2, du3, d1, d2, d3]) if (d == 255 or d > 400)]
                    self.get_logger().warn(f"Not publishing! Invalid senors: {invalid}")

        except Exception as e:
            self.get_logger().warn(f"Error reading serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VL6180MultiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Range
# from std_msgs.msg import Header
# import serial
# import time

# class VL6180Node(Node):
#     def __init__(self):
#         super().__init__('vl6180_sensor_node')

#         # Ajusta este puerto si tu Arduino está en otro
#         self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

#         self.publisher_ = self.create_publisher(Range, 'vl6180/range', 10)

#         # Publicar a 10 Hz
#         self.timer = self.create_timer(0.1, self.read_serial_data)

#         self.get_logger().info("VL6180 node using sensor_msgs/Range initialized.")

#     def read_serial_data(self):
#         try:
#             # Limpiar todos los datos menos el último
#             while self.serial.in_waiting > 0:
#                 last_line = self.serial.readline()

#             # Procesar solo la última línea leída
#             line = last_line.decode('utf-8').strip()

#             if line.isdigit():
#                 distance_mm = int(line)
#                 distance_m = distance_mm / 1000.0

#                 msg = Range()
#                 msg.header = Header()
#                 msg.header.stamp = self.get_clock().now().to_msg()
#                 msg.header.frame_id = "vl6180_sensor"

#                 msg.radiation_type = Range.INFRARED
#                 msg.field_of_view = 0.0349
#                 msg.min_range = 0.01
#                 msg.max_range = 0.18
#                 msg.range = min(max(distance_m, msg.min_range), msg.max_range)

#                 self.publisher_.publish(msg)
#                 self.get_logger().info(f"Published range: {msg.range:.3f} m")

#         except Exception as e:
#             self.get_logger().warn(f"Serial read error: {e}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = VL6180Node()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
