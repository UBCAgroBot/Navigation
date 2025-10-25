import rclpy
from rclpy.node import Node

# Get ROS2 messages
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDriveStamped

# Processing libraries
import math
import numpy as np

class gap_follower(Node):
    def __init__(self):
        super().__init__('gap_follower')
        self.get_logger().info('==========Gap Following node started==========')

        self.create_subscription(OccupancyGrid, '/slam', self.gap, 10)

    def gap(self, map: OccupancyGrid):
        self.get_logger().info(f'Map received')

def main(args=None):
    rclpy.init()
    node = gap_follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()