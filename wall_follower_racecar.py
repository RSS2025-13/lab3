#!/usr/bin/env python3
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        #TODO change these defaults to pull from the yaml file
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/high_level/input/nav_0")
        self.declare_parameter("side", -1)
        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("desired_distance", 1.)
        self.declare_parameter("dt", 0.05)

        # TODO Fetch constants from the ROS parameter server
        # This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
        self.DT = self.get_parameter('dt').get_parameter_value().double_value

        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS! 
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.wall_dist = self.DESIRED_DISTANCE #TODO better default?
        self.wall_slope = 100.0
        self.kp = 10
        self.kd = 3
		
        self.scan_subscriber = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.scan_callback, 10)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        #self.dist_publisher = self.create_publisher(Float32, '/dist', 10)
        self.timer = self.create_timer(self.DT, self.main_callback)
        self.get_logger().info(f'Initiated wall following node {self.SIDE}, speed: {self.VELOCITY}')
    
    #revolution:
    # - cut off forward points if they are greater than 1.x distance away 
    # - use all points to generate line
    # - actually find distance to line
    # - slope in derivative
    # piazza fix to change side param
    # run simulator to confirm
    # run tests
    def main_callback(self):
        error = self.SIDE*(self.DESIRED_DISTANCE - abs(self.wall_dist))
        slope_term = 1/(round(self.wall_slope,1)+0.1)

        #PD
        steering_angle = round(-(2*math.pi/180*self.kp*error) + self.kd*slope_term, 1)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = self.VELOCITY
        self.drive_publisher.publish(drive_msg)
        # dist_msg = Float32()
        # dist_msg.data = self.wall_slope
        # self.dist_publisher.publish(dist_msg)

    def scan_callback(self, scan:LaserScan):
        #process scan to create list of right/left side data points, and front wall points
        ranges = np.array(scan.ranges) #angle min to angle max, CCW
        angle_min, angle_max = scan.angle_min, scan.angle_max #about -134 to 134

        edge_bound = 90*math.pi/180 #abs values
        front_bound = 45*math.pi/180 #abs values

        saved_range = ranges
        saved_angles = np.linspace(angle_min, angle_max, len(saved_range)) #TODO off by 1 error?
        #first, get rid of far away points beyond 2x des dist
        trim1 = saved_range > 4*self.DESIRED_DISTANCE
        saved_range = saved_range[np.logical_not(trim1)]
        saved_angles = saved_angles[np.logical_not(trim1)]
        #then, trim ranges to be bounded by edge, front
        if self.SIDE == -1: #right
            keep = np.logical_and(saved_angles >= -edge_bound, saved_angles <= front_bound)
            saved_range = saved_range[keep]
            saved_angles = saved_angles[keep]
        else: #side is 1, left
            keep = np.logical_and(saved_angles >= -front_bound, saved_angles <= edge_bound)
            saved_range = saved_range[keep]
            saved_angles = saved_angles[keep]
        #next, threshold trim values from front, so they only show up within 1.2 des dist
        trim2 = np.logical_and(saved_angles >= -front_bound, np.logical_and(saved_angles <= front_bound, saved_range > 1.2*self.DESIRED_DISTANCE))
        saved_range = saved_range[np.logical_not(trim2)] #save to global
        saved_angles = saved_angles[np.logical_not(trim2)]

        #self.get_logger().info(f'f, s: {len(front_range)}, {len(side_range)}')
        if len(saved_range) > 1: #if there are points to interpolate
            #linear fit to find equations of wall (angled on turn)
            x = saved_range*np.sin(saved_angles) #element-wise multiply
            y = saved_range*np.cos(saved_angles)
            self.wall_slope, y_i = np.polyfit(x, y, 1)
            x_i = -y_i/self.wall_slope
            self.wall_dist = round(x_i*abs(y_i)/math.sqrt(x_i**2+y_i**2),1) #shortest distance to wall line
            self.wall_slope = round(self.wall_slope,1)
        else: #otherwise, turn towards the side you are trying to follow 
            #controls intensity of turn
            self.wall_dist = self.SIDE*2*self.DESIRED_DISTANCE
            self.wall_slope = self.SIDE*1.0
            x, y = [], []
            x_i, y_i = None, None
        
        self.get_logger().info(f'dist: {self.wall_dist}, ws: {self.wall_slope}')
    
    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!
        
        This is used by the test cases to modify the parameters during testing. 
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'side':
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
