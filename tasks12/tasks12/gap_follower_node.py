import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class gap_follower_node(Node):
    def __init__(self):
        super().__init__('gap_follower_node')

        self.speed = 1.0                 # m/s
        self.min_dist = 0.3              # obstacle threshold (meters)
        self.max_range = 5.0             # meters
        self.max_steer = math.radians(35)

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.steer_fl = self.create_publisher(Float64, '/steer_fl', 10)
        self.steer_fr = self.create_publisher(Float64, '/steer_fr', 10)

        self.get_logger().info("Gap Follower started")

    def scan_callback(self, scan: LaserScan):
        ranges = list(scan.ranges)

        # Clean scan
        for i in range(len(ranges)):
            if not math.isfinite(ranges[i]) or ranges[i] > self.max_range:
                ranges[i] = self.max_range
            elif ranges[i] < 0.01:
                # Set as obstacle
                ranges[i] = 0.0

        # Block nearby obstacles
        for i in range(len(ranges)):
            if ranges[i] < self.min_dist:
                ranges[i] = 0.0

        # Find largest gap
        best_start = 0
        best_end = best_start
        best_width = 0

        i = 0
        while i < len(ranges):
            while i < len(ranges) and ranges[i] == 0.0:
                i += 1
            start = i

            while i < len(ranges) and ranges[i] > 0.0:
                i += 1
            end = i

            if end - start > best_width:
                best_width = end - start
                best_start = start
                best_end = end

        # Stop if no gap
        if best_width <= 1:
            self.publish_drive(0.0, 0.0)
            return

        # Drive to center of gap
        center_index = (best_start + best_end) // 2
        steering_angle = scan.angle_min + center_index * scan.angle_increment
        steering_angle = max(-self.max_steer,
                              min(self.max_steer, steering_angle))

        self.publish_drive(self.speed, steering_angle)

    def publish_drive(self, speed, steering):
        msg = Twist()
        msg.linear.x = speed
        self.cmd_pub.publish(msg)

        steer_msg = Float64()
        steer_msg.data = steering
        self.steer_fl.publish(steer_msg)
        self.steer_fr.publish(steer_msg)


def main():
    rclpy.init()
    node = gap_follower_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
