import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class gap_follower_node(Node):
    """
    Gap Follower Node
    
    This node implements a gap-following algorithm that:
    1. Processes lidar data to find safe gaps
    2. Creates safety bubbles around obstacles
    3. Extends disparities to account for vehicle width
    4. Uses PD control to steer toward the largest safe gap
    5. Adjusts speed based on steering angle for safety
    """

    def __init__(self):
        """
        Initialize the gap follower node.
        
        Sets up subscriptions, publishers, and control parameters.
        Configures PD controller gains and safety thresholds.
        """

        super().__init__('gap_follower_node')

        self.get_logger().info('==========Gap Follower Node initialized===========')

        self.car_width = 0.5                # meters        (width of the car)
        self.Kp = 0.55                      # dimensionless (porportional control)
        self.Kd = 1.1                       # dimensionless (derivative control)
        self.speed = 0.0                    # meters/second (speed to be published) 
        self.max_distance_threshold = 2.5   # meters   

        self.prev_error = 0.0               # radians
        self.prev_steering_angle = 0.0      # radians
        self.distance_threshold = 2.0       # meters
        self.disparity_threshold = 0.5      # meters

        self.ebrake = False                 # activates when autobraking is enabled
        self.no_gap_detected = False        # activates when no gap is detected

        self.create_subscription(LaserScan, '/scan', self.follow_gap, 10)
        self.create_subscription(Bool, '/brake', self.emergency_brake, 10)

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.steer_fl = self.create_publisher(
            Float64,
            '/steer_fl',
            10
        )
    
        self.steer_fr = self.create_publisher(
            Float64,
            '/steer_fr',
            10
        )
        # self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        #self.speed_pub = self.create_publisher(Float32, '/speed', 10)


    def follow_gap(self, scan: LaserScan):
        """
        This function is the main function ran when /scan is published to by the Lidar. We first fix the
        max distance that the lidar can see, and then create a safety bubble around the closest 
        obstacle/wall to the car. Then we extend the disparity to create a safety margin for the car to 
        drive through. Then we find the largest gap visible to the lidar and select the best steering 
        angle to drive the car to the center of that gap, adjusting the speed of the car based on how
        extreme the steering angle is to prioritize safety.

        In any scenario where a crash is imminent or no gap is found, we fallback to the AEB.

        param LaserScan: Scan from the Lidar of the car. This scan will be used to find and follow gaps.

        returns: N/A.
        """

        # Step 1: Clean the raw LiDAR data
        fixed_scan = self.remove_invalid_values(scan)

        # Step 2: Create safety bubble around closest obstacle
        fixed_scan = self.bubble_around_closest_ray(fixed_scan, scan)

        # Step 3: Extend disparities to account for vehicle width
        fixed_scan = self.disparity_extender(fixed_scan, scan)

        # Stop moving if no gap is found anywhere (Lidar can't read anything meaningful)
        if (max(fixed_scan) <= 1.1):
            msg = Twist()
            msg.linear = 0.0
            steering_ang = Float64()
            steering_ang.data = scan.angle_min + (192/2) * scan.angle_increment
            self.cmd_pub.publish(msg)
            self.steer_fl.publish(steering_ang) 
            self.steer_fr.publish(steering_ang)  
            return

        # Step 4: Find center of largest safe gap
        center_index, center_angle = self.find_center_largest_gap(fixed_scan, scan)

        self.no_gap_detected = False    # Default: Gap was detected

        # When no gap is found, keep the car straight for the current timestep
        if center_index is None:
            center_index = len(scan.ranges) // 2
            center_angle = scan.angle_min + center_index * scan.angle_increment
            self.no_gap_detected = True

        # Step 5: Calculate steering error and apply PD control
        error = center_angle
        steering_angle = self.Kp*error + self.Kd*(self.prev_error - error)

        # Adjust speed based on how extreme the steering angle is
        if abs(steering_angle) > math.radians(30.0) or self.no_gap_detected:    # Slow down if no gap detected
            # Min speed
            self.speed = 0.3
        elif abs(steering_angle) > math.radians(25.0):
            self.speed = 0.5
        elif abs(steering_angle) > math.radians(10.0):
            self.speed = 0.9
        else:
            # Max speed
            self.speed = 1.6

        # msg_speed = Float32()
        # msg_speed.data = self.speed
        # self.speed_pub(msg_speed)


        # If no brake override from AEB
        if not self.ebrake:
            msg = Twist()
            msg.data = self.speed
            steer_ang = Float64()
            steer_ang.data = steering_angle
            
            self.steer_fl.publish(steer_ang)
            self.steer_fr.publish(steer_ang)

            self.cmd_pub.publish(msg)

        # if (self.no_gap_detected):
        #     self.get_logger().info('No gap detected. Going center')
        # else:
        #     self.get_logger().info(f'Steering angle: {steering_angle*180/math.pi}')


        # Update for next scan
        self.prev_error = error
        self.prev_steering_angle = steering_angle

    
    def remove_invalid_values(self, scan):
        """
        Adjust LiDAR scan data by replacing invalid values.
        
        Handles common LiDAR issues:
        - Infinite/NaN values
        - Values exceeding maximum threshold (too far)
        - Negative/zero values (invalid measurements)
        
        Params:
            scan (LaserScan): Raw lidar scan data
            
        Returns:
            list: Cleaned scan data with finite values only
        """

        scan_fixed = list(scan.ranges)

        # Iterate through all scan distance values and cap them between 0 and the maximum distance
        for i in range(len(scan_fixed)):
            # Replace infinite values and distances beyond threshold
            if (not math.isfinite(scan_fixed[i])) or scan_fixed[i] > self.max_distance_threshold:
                scan_fixed[i] = self.max_distance_threshold
            # Replace negative/zero values with maximum range
            elif  scan_fixed[i] <= 0.0:
                scan_fixed[i] = scan.range_max
        return scan_fixed


    def bubble_around_closest_ray(self, scan_fixed, scan):
        """
        Create a safety bubble around the closest detected obstacle.
        
        This prevents the vehicle from getting too close to obstacles by
        treating the area around the closest point as "blocked" space.
        
        Params:
            scan_fixed (list): preprocessed scan data
            scan (LaserScan): original scan data for angle calculations
            
        Returns:
            list: scan data with safety bubble applied
        """

        # Find the closest obstacle (or wall)
        min_distance = min(scan_fixed)
        min_index = scan_fixed.index(min_distance)

        # Calculate half-angle needed for car-width safety bubble
        # This ensures the vehicle can't get closer than half its width to any obstacle
        if min_distance > 0:
            # After comparing asin and atan, asin gave better performance
            half_angle = math.asin(min((self.car_width / 2.0) / min_distance, 1.0))
        else:
            half_angle = math.pi / 2

        # Convert angle to index range with 28% safety margin
        bubble_buffer = 1.28
        k = int(bubble_buffer * half_angle / scan.angle_increment)

        # Define bubble boundaries
        left = max(0, min_index - k)
        right = min(len(scan_fixed), min_index + k + 1)

        # Mark all points inside bubble as blocked (distance = 0)
        for j in range(left, right):
            scan_fixed[j] = 0.0

        return scan_fixed

    def disparity_extender(self, scan_fixed, scan):
        """
        Extend detected disparities to account for vehicle width.
        
        When navigating around obstacles, sudden distance changes (disparities)
        need to be extended to ensure the vehicle doesn't clip obstacles.
        
        Params:
            scan_fixed (list): scan data with safety bubble applied
            scan (LaserScan): original scan data for angle calculations
            
        Returns:
            list: scan data with disparities extended
        """

        extended_scan = scan_fixed.copy()
        
        for i in range(1, len(scan_fixed) - 1):
            prev_dist = scan_fixed[i - 1]
            curr_dist = scan_fixed[i]
            next_dist = scan_fixed[i + 1]
            
            # Skip if any adjacent point is blocked or in the safety bubble
            if prev_dist == 0.0 or curr_dist == 0.0 or next_dist == 0.0:
                continue
                
            # Checking for disparity to the right
            if curr_dist < next_dist - self.disparity_threshold:
                # Calculate extension angle needed for vehicle width
                extension_angle = math.asin(min((self.car_width / 2.0) / curr_dist, 1.0))
                right_disparity_scale = 1.1
                extension_steps = (int)((right_disparity_scale)*(extension_angle / scan.angle_increment))
                
                # Extend the disparity to the right
                for j in range(i, min(i + extension_steps + 1, len(extended_scan))):
                    extended_scan[j] = curr_dist
                    
            # Check for disparity to the left
            if curr_dist < prev_dist - self.disparity_threshold:
                extension_angle = math.asin(min((self.car_width / 2.0) / curr_dist, 1.0))
                left_disparity_scale = 1.1
                extension_steps = (int) ((left_disparity_scale)*(extension_angle / scan.angle_increment))
                
                # Extend the disparity to the left
                for j in range(max(0, i - extension_steps), i + 1):
                    extended_scan[j] = curr_dist
        
        return extended_scan

    def find_center_largest_gap(self, scan_fixed, scan):
        """
        Find the center of the largest safe gap within the field of view.
        Searches for continuous regions where the distance exceeds the minimum
        threshold, then returns the center of the widest possible window.
        
        Params:
            scan_fixed (list): processed scan data with bubbles and disparities
            scan (LaserScan): Original scan data for angle calculations
            
        Returns:
            tuple : (center_index, center_angle) or (None, None) if no gap found
                - center_index (int): Array index of gap center
                - center_angle (float): Angular position of gap center (radians)
        """

        start_index = 0
        end_index = 0

        min_threshold = min(self.distance_threshold, scan.range_max)

        # Define field of view, This limits search to the forward-facing area (will not check behind car)
        i_min = 0     # Min angle considered = -85deg in Lidar
        i_max = 192     # Max angle considered = +85deg in Lidar
        best_start, best_end, best_width = 0, 0, 0

        # Search for continuous gaps
        i = i_min
        while i <= i_max:
            # Skip over blocked areas 
            while i <= i_max and scan_fixed[i] < min_threshold:
                i += 1
            start_index = i

            # Find end of current gap
            while i <= i_max and scan_fixed[i] >= min_threshold:
                i += 1
            end_index = i

            # Check if this is the widest gap found so far
            width = end_index - start_index
            if start_index < end_index and width > best_width:
                best_width = width
                best_start = start_index
                best_end = end_index

        # Return None if no valid gap found
        if best_width == 0:
            return None, None
        
        # Calculate center of best gap
        center_index = (best_start + best_end) // 2
        center_angle = scan.angle_min + center_index * scan.angle_increment

        return center_index, center_angle



    def emergency_brake(self, brake: Bool):
        """
        Handle emergency brake signals from the safety system.
        
        When emergency brake is activated, this prevents the gap follower
        from publishing drive commands, allowing the safety system to take over.
        
        Params:
            brake (Bool): emergency brake status message from safety node
        """

        self.ebrake = brake.data # Update emergency brake flag

            
def main():
    rclpy.init()
    gf_node =  gap_follower_node()
    rclpy.spin(gf_node)
    gf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
