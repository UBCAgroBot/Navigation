import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class demo_lidar(Node):
    def __init__(self):
        super().__init__('demo_lidar')

        self.get_logger().info('==========Fake lidar data node initialized===========')

        self.pub = self.create_publisher(LaserScan, '/scan', 10)

        timer_period = 2
        self.timer = self.create_timer(timer_period, self.generate_lidar_data)

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
        rays_in_scan = 192
        scan = [0] * rays_in_scan

        #set middle distance
        midright = int(rays_in_scan/2 - 1)
        midleft = int(rays_in_scan/2)

        #right side
        rightmost = 0.25/math.sin(math.radians(60.0))
        rightincrement = (10.0 - rightmost)/(rays_in_scan/2)
        for i in range(0, int(rays_in_scan/2)):
            scan[i] = rightmost + i*rightincrement

        #left side
        leftmost = 0.5/math.sin(math.radians(60.0))
        leftdecrement = (10.0 - leftmost)/(rays_in_scan/2)
        for j in range(0, int(rays_in_scan/2)):
            scan[j+midleft] = 10.0 - (j+1)*leftdecrement

        # self.get_logger().info(f'Scan: {scan}')
        self.get_logger().info(f'Distance in the middle: {scan[int(len(scan)/2)]}')
        self.get_logger().info(f'Distance at the left: {scan[-1]}')
        self.get_logger().info(f'Distance at the right: {scan[0]}')        

        lidar_msg.ranges = scan
        self.pub.publish(lidar_msg)

        
def main():
    rclpy.init()
    lidar_node = demo_lidar()
    rclpy.spin(lidar_node)
    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()