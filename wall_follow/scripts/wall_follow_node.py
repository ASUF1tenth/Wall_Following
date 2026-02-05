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

        # TODO: create subscribers and publishers

        # TODO: set PID gains
        # self.kp = 
        # self.kd = 
        # self.ki = 

        # TODO: store history
        # self.integral = 
        # self.prev_error = 
        # self.error = 

        # TODO: store any necessary values you think you'll need

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        return 0.0

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        # Use two rays to estimate wall angle and distance (left wall)
        # angles measured in radians; ROS LaserScan angles increase counter-clockwise
        # choose a = 90deg (left) and b = 45deg (front-left)
        a = np.pi / 2.0
        b = np.pi / 4.0

        # get distances along the two rays
        d_a = self.get_range(range_data, a)
        d_b = self.get_range(range_data, b)

        # protect against degenerate measurements
        if d_a <= 0.0:
            d_a = 10.0
        if d_b <= 0.0:
            d_b = 10.0

        # phi is the angle between the two rays
        phi = a - b

        # compute orientation alpha of the wall relative to the car
        # formula from common two-ray wall-following approach
        denom = (d_a * np.sin(phi))
        if abs(denom) < 1e-6:
            alpha = 0.0
        else:
            alpha = np.arctan((d_a * np.cos(phi) - d_b) / denom)

        # distance from car to wall perpendicular to car axis
        distance_to_wall = d_b * np.cos(alpha)

        # error: positive when we are farther than desired (need to go left),
        # negative when we are too close (need to steer right)
        error = dist - distance_to_wall
        return float(error)

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
        error = 0.0 # TODO: replace with error calculated by get_error()
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