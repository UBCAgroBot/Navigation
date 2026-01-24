import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class gap_follower_node(Node):
    def __init__(self):
        super().__init__('simple_gap_follower')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.forward_speed = 0.3      # m/s
        self.stop_distance = 0.8      # meters

        self.get_logger().info('Simple Gap Follower started')

    def lidar_callback(self, scan: LaserScan):
        # Look at a small window in front of the robot
        num_ranges = len(scan.ranges)
        center_index = num_ranges // 2
        window = 20  # +- 20 samples around center

        front_ranges = scan.ranges[
            center_index - window : center_index + window
        ]

        # Filter invalid readings
        front_ranges = [
            r for r in front_ranges
            if not math.isinf(r) and not math.isnan(r)
        ]

        cmd = Twist()

        if front_ranges and min(front_ranges) > self.stop_distance:
            # Free space ahead → move forward
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
        else:
            # Obstacle too close → stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = gap_follower_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
