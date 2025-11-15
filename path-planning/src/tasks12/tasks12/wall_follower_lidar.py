import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool

class wall_follower_lidar(Node):
    def __init__(self):
        super().__init__('wall_follower_lidar')

        self.get_logger().info('==========Lidar Wall Follower Node initialized===========')

        self.theta = 50.0 
        self.good_distance = 0.5

        # Controller gains
        self.Kp = 1.0  # Proportional gain
        self.Kd = 0.1  # Derivative gain

        # Speed control
        self.speed = 1.0
        self.AC = 0.3    # basically lookahead distance (expected dist travelled between scans)
        
        # Control state
        self.prev_error = 0.0
        self.prev_steering_angle = 0.0

        # ROS2 setup
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(LaserScan, '/scan', self.follow_wall, 10)

    def follow_wall(self, scan: LaserScan):
        scanl = list(scan.ranges)
        """Main wall following algorithm using lookahead distance method."""
        angle_range_rad = math.radians(120.0)  # Total Lidar field of view
        theta_rad = math.radians(self.theta) 

        index_b = int(192*(math.pi/2 - scan.angle_min)/angle_range_rad)  
        index_a = index_b - int(192*theta_rad/angle_range_rad)  

        b = scan.ranges[index_b] 
        a = scan.ranges[index_a] 

        # Calculate current wall distance
        wall_distance = a * math.sin(theta_rad)

        # Calculate wall angle and predicted distance
        alpha = math.atan2(a * math.cos(theta_rad) - b, wall_distance) 
        AB = b * math.cos(alpha)
        CD = AB + self.AC * math.sin(alpha)  # Predicted distance at future point

        # Error calculation (opposite of lecture slides because this follows left wall)
        current_error = CD - self.good_distance

        # PD control for steering
        steering_angle = self.Kp * current_error + self.Kd * (self.prev_error - current_error)

        # Simple distance-based brake
        if (max(scanl) <= 1.1):
            msg = AckermannDriveStamped()
            msg.drive.steering_angle = scan.angle_min + (192/2) * scan.angle_increment
            msg.drive.speed = 0.0
            self.pub.publish(msg)
            return

        if abs(steering_angle) > math.radians(25.0):
            self.speed = 0.7
        elif abs(steering_angle) > math.radians(10.0):
            self.speed = 0.8
        else:
            self.speed = 1.0


        # Create and publish drive command
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = self.speed

        # Update state for next iteration
        self.prev_error = current_error
        self.prev_steering_angle = steering_angle
        
def main():
    rclpy.init()
    wf_node = wall_follower_lidar()
    rclpy.spin(wf_node)
    wf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()