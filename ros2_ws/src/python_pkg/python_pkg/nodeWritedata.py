import rclpy
from rclpy.node import Node
from subsonus_pkg.msg import SubsonusRawSensorsPacket  # Replace with your custom message type
import os

class MagneticDataLogger(Node):

    def __init__(self):
        super().__init__('nodeWritedata')
        self.subscription = self.create_subscription(
            SubsonusRawSensorsPacket,  # Replace with your custom message type
            '/subsonus/raw_sensors_packet_surface',  # Replace with the actual topic name
            self.magnetic_data_callback,
            10)
        self.file_path = 'raw_data_magneto_surface_11_24.txt'  # Replace with the output file name
        self.file = open(self.file_path, 'a')

    def magnetic_data_callback(self, msg):
        # Extract magnetic data (x, y, z) from the received message
        x = msg.magnetometer_x
        y = msg.magnetometer_y
        z = msg.magnetometer_z

        # Write the data to the file
        self.file.write(f"{x} {y} {z}\n")

    def shutdown(self):
        # Close the file when the node is shutdown
        self.file.close()

def main(args=None):
    rclpy.init(args=args)
    node = MagneticDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
