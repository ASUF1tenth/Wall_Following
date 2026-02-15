import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # create subscribers and publishers
        self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # set PID gains
        self.kp = 0.8
        self.kd = 0.15
        # self.ki = 0

        # store history
        # self.integral = 
        self.prev_error = 0.0
        self.error = 0.0

        # TODO: store any necessary values you think you'll need
        self.MAX_LIDAR_DIST = 10.0  # Maximum valid LiDAR distance in meters
        self.range_offset = 180

    def preprocess_lidar(self, ranges):
        """Preprocess the LiDAR scan array with smoothing.
        
        Args:
            ranges: raw range array from LiDAR
            
        Returns:
            proc_ranges: preprocessed and smoothed range array
        """
        proc_ranges = np.array(ranges)
        proc_ranges = np.array(ranges[self.range_offset:-self.range_offset])
        # Replace inf and nan values with maximum distance
        proc_ranges[np.isinf(proc_ranges)] = self.MAX_LIDAR_DIST
        proc_ranges[np.isnan(proc_ranges)] = self.MAX_LIDAR_DIST
        
        # Apply moving average filter for smoothing (window size of 5)
        window_size = 3
        kernel = np.ones(window_size) / window_size
        proc_ranges = np.convolve(proc_ranges, kernel, mode='valid')
        
        # Cap maximum distance
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        
        return proc_ranges

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        # Get the angular increment (delta) from the LiDAR
        delta = 0.00436332312998582
        # Choose angle θ for beam a (theta = 2 * delta in your specification)
        # You can adjust the multiplier as needed
        theta = 20 * delta
        
        # Beam b at 90° to the right (-90° or -π/2 radians)
        angle_b = np.radians(-90.0)
        
        # Beam a at angle θ ahead of beam b
        angle_a = angle_b + theta

        # Read distances from LiDAR using preprocessed data
        b = range_data[0]
        a = range_data[20]

        
        # Compute the wall angle α
        alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
        
        # Compute the current distance to the wall
        Dt = b * np.cos(alpha)
        
        # Choose a lookahead distance L
        L = 1.0  # meters
        
        # Compute the future distance to the wall
        D_future = Dt + L * np.sin(alpha)
        
        # Compute the error (dist is the desired distance)
        self.prev_error = self.error
        error = dist - D_future
        self.error = error
        self.get_logger().info(f"{a},{b},{error}")
        return error

    def get_speed(self, steering_angle):
        # ===== Speed Control =====
        if abs(steering_angle) > 0.4:
            speed = 1.0
        elif abs(steering_angle) > 0.2:
            speed = 2.0
        else:
            speed = 3.0
        return speed

    def pid_control(self, error):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error

        Returns:
            None
        """
        # Use kp, ki & kd to implement a PID controller
        steering_angle =  self.kp * error + self.kd * (error - self.prev_error)

        return steering_angle

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """

        # preprocess lidar ranges
        preprocessed_ranges = self.preprocess_lidar(msg.ranges)
    
        # Set desired distance to wall
        width = preprocessed_ranges[0] + preprocessed_ranges[len(preprocessed_ranges)-1]
        desired_distance = width/2  # meters
        
        # Calculate error using get_error with preprocessed_ranges and desired_distance
        error = self.get_error(preprocessed_ranges, desired_distance)
        
        # calculate steering angle with PID
        steering_angle = self.pid_control(error)

        # calculate desired car velocity based on steering angle
        speed = self.get_speed(steering_angle)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed

        self.drive_pub.publish(drive_msg)


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