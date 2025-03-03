#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    WALL_TOPIC = "/wall"

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS! 
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("desired_distance", "default")

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
		
        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS! 
        self.add_on_set_parameters_callback(self.parameters_callback)
  
        # TODO: Initialize your publishers and subscribers here
        self.line_pub = self.create_publisher(Marker, self.WALL_TOPIC, 1)

        self.scan_subscriber = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.listener_callback, 10)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)

        self.previous_error = 0
        self.previous_time = self.get_clock().now()
        self.left_or_right = self.SIDE

        self.kp = 4
        self.kd = 2
        self.ki = 0

        self.get_logger().info(f'kp: "{self.kp}"')
        self.get_logger().info(f'kd = "{self.kd}"')

        # TODO: Write your callback functions here

    def listener_callback(self, msg):
        ads_msg = AckermannDriveStamped()

        now = self.get_clock().now()

        ads_msg.header.stamp = now.to_msg()
        ads_msg.header.frame_id = 'base_link'
        
        
        ads_msg.drive.speed = self.VELOCITY
        #ads_msg.drive.steering_angle = np.pi/4
        ads_msg.drive.steering_angle = self.PID_angle(laser_msg = msg, current_time = now)

        self.drive_publisher.publish(ads_msg)

        #self.get_logger().info(f'min angle: "{msg.angle_min}" and max_angle: "{msg.angle_max}"')
        #self.get_logger().info(f'published angle "{ads_msg.drive.steering_angle}"and speed "{ads_msg.drive.speed}"')

    def PID_angle(self, laser_msg, current_time):
        #Creating array of desired values
        ranges_array = np.array(laser_msg.ranges)   #converts ranges array to numpy array for manipulation
        min_angle_sweep = -3*np.pi/4   #-2*np.pi/3
        max_angle_sweep =  np.pi/12 #-np.pi/6  #Gets the max angle to sweep to (min is -3pi/4)
        
        number_of_elements_sweeped = int((max_angle_sweep - min_angle_sweep) / laser_msg.angle_increment) + 1   #determines the number of values within that sweep given the angle increment
        first_index = int(np.abs(laser_msg.angle_min - min_angle_sweep) / laser_msg.angle_increment)
        last_index = first_index + number_of_elements_sweeped
        side_ranges_array = ranges_array[:last_index] if self.SIDE == -1 else ranges_array[-last_index:]     #Creates an array of just the desired values from that side

        #creates x and y values from each Lidar Scan point
        thetas = np.arange(min_angle_sweep, max_angle_sweep, laser_msg.angle_increment) if self.SIDE == -1 else np.arange(-1 * max_angle_sweep, -1 * min_angle_sweep, laser_msg.angle_increment)
        x_array = side_ranges_array * np.cos(thetas)
        y_array = side_ranges_array * np.sin(thetas)

        #create a LSR 

        #limiting values with r > 4m
        mask = side_ranges_array <= 3  # Boolean mask where values are <= 4
        new_x_array = x_array[mask]
        new_y_array = y_array[mask]

        coeffs = np.polyfit(new_x_array, new_y_array, 1)

        #Find the shortest distance between the line and the car
        d = np.abs(coeffs[1]) / np.sqrt(1 + (coeffs[0]**2))

        #Determine error and PID controller
        error = (self.DESIRED_DISTANCE - d) * (-1 if self.SIDE == 1 else 1)

        #self.left_or_right = (-1 if self.previous_error <= error else 1) * self.left_or_right
        delta_e = error - self.previous_error
        delta_t = (current_time - self.previous_time).nanoseconds / 1e9

        proportional = self.kp * error

        derivative = self.kd * (delta_e / delta_t)

        integral = self.ki * (error * delta_t)

        u_t = (proportional + derivative + integral) #* self.left_or_right

        self.previous_time = current_time
        self.previous_error = error

        #Visualization
        y_visual = coeffs[0] * new_x_array + coeffs[1]
        VisualizationTools.plot_line(new_x_array, y_visual, self.line_pub, frame="/laser")

        return u_t
    
    
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
    