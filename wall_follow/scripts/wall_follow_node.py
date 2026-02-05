import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
#perception
#

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        # TODO: set PID gains
        # self.kp = 
        # self.kd = 
        # self.ki = 

        # TODO: store history
        # self.integral = 
        # self.prev_error = 
        # self.error = 

        # TODO: store any necessary values you think you'll need
        self.MAX_LIDAR_DIST = 10.0  # Maximum valid LiDAR distance in meters

    def preprocess_lidar(self, ranges):
        """Preprocess the LiDAR scan array with smoothing.
        
        Args:
            ranges: raw range array from LiDAR
            
        Returns:
            proc_ranges: preprocessed and smoothed range array
        """
        proc_ranges = np.array(ranges)
        
        # Replace inf and nan values with maximum distance
        proc_ranges[np.isinf(proc_ranges)] = self.MAX_LIDAR_DIST
        proc_ranges[np.isnan(proc_ranges)] = self.MAX_LIDAR_DIST
        
        # Apply moving average filter for smoothing (window size of 5)
        window_size = 5
        kernel = np.ones(window_size) / window_size
        proc_ranges = np.convolve(proc_ranges, kernel, mode='same')
        
        # Cap maximum distance
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        
        return proc_ranges
    
    def get_range(self, range_data, angle, preprocessed_ranges=None):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR
            preprocessed_ranges: optional preprocessed range array

        Returns:
            range: range measurement in meters at the given angle

        """
        # Calculate the index corresponding to the angle
        index = int((angle - range_data.angle_min) / range_data.angle_increment)
        
        # Ensure index is within bounds
        ranges_to_use = preprocessed_ranges if preprocessed_ranges is not None else range_data.ranges
        if index < 0 or index >= len(ranges_to_use):
            return 0.0
        
        # Get the range value
        range_val = ranges_to_use[index]
        
        # Handle NaN and inf values (should already be handled if preprocessed)
        if np.isnan(range_val) or np.isinf(range_val):
            return self.MAX_LIDAR_DIST
        
        return range_val

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        # Preprocess LiDAR data for smoothing
        preprocessed_ranges = self.preprocess_lidar(range_data.ranges)
        
        # Get the angular increment (delta) from the LiDAR
        delta = range_data.angle_increment
        
        # Choose angle θ for beam a (theta = 2 * delta in your specification)
        # You can adjust the multiplier as needed
        theta = 2 * delta
        
        # Beam b at 90° to the right (-90° or -π/2 radians)
        angle_b = np.radians(90.0)
        
        # Beam a at angle θ ahead of beam b
        angle_a = angle_b + theta
        
        # Read distances from LiDAR using preprocessed data
        b = self.get_range(range_data, angle_b, preprocessed_ranges)
        a = self.get_range(range_data, angle_a, preprocessed_ranges)
        
        # Compute the wall angle α
        alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
        
        # Compute the current distance to the wall
        Dt = b * np.cos(alpha)
        
        # Choose a lookahead distance L
        L = 1.0  # meters
        
        # Compute the future distance to the wall
        D_future = Dt + L * np.sin(alpha)
        
        # Compute the error (dist is the desired distance)
        error = dist - D_future
        
        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        # Set desired distance to wall
        desired_distance = 1.0  # meters
        
        # Calculate error using get_error with msg (which contains angle_increment, ranges, etc.)
        error = self.get_error(msg, desired_distance)
        
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity) # TODO: actuate the car with PID


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()