import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class forward_node(Node):
    def __init__(self):
        super().__init__('forward_node')

        self.get_logger().info('========== Simple Forward Twist Node Initialized ==========')

        # Subscribe to LiDAR (only to trigger callback)
        self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10
        )

        # Publisher for Twist commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Constant forward velocity
        self.linear_speed = 1.0   # m/s
        self.angular_speed = 0.0  # rad/s

    def lidar_callback(self, scan: LaserScan):
        # Print on every LiDAR callback
        print("lidar...")

        # Publish constant forward motion
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_speed  # 0 = straight

        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = forward_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
