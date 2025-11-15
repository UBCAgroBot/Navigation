import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class demo_lidar(Node):
    def __init__(self):
        super().__init__('demo_lidar')

        self.get_logger().info('==========Fake lidar data node initialized===========')

        self.pub = self.create_publisher(LaserScan, '/scan', 10)

    def generate_lidar_data(self):
        # Assumes only horizontal scans are received
        lidar_msg = LaserScan()
        lidar_msg.angle_min = math.radians(-60.0)
        lidar_msg.angle_max = math.radians(60.0)
        lidar_msg.angle_increment = math.radians(0.625)
        lidar_msg.time_increment = float(1/260000)
        lidar_msg.scan_time = 0.1
        lidar_msg.range_min = 0.1
        lidar_msg.range_max = 75.0
        lidar_msg.intensities = []

        # Generate scan
        scan = []
        rays_in_scan = 192


        lidar_msg.ranges = scan

        
def main():
    rclpy.init()
    lidar_node = demo_lidar()
    rclpy.spin(lidar_node)
    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()