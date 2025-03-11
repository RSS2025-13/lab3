#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
from robot_wall_follower.evaluations import Evaluations
from robot_wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    WALL_TOPIC = "/wall"

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS! 
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/high_level/input/nav_0")
        self.declare_parameter("side", -1)
        self.declare_parameter("velocity", 0.5)
        self.declare_parameter("desired_distance", 0.5)
        self.declare_parameter("kp", 0.6)
        self.declare_parameter("kd", 0.0)
        self.declare_parameter("ki", 0.0)

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
        self.score_pub = self.create_publisher(Float32, 'score', 1)

        self.scan_subscriber = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.listener_callback, 10)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)

        self.previous_error = 0
        self.previous_time = self.get_clock().now()
        self.left_or_right = self.SIDE

        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value

        self.get_logger().info(f'kp: "{self.kp}"')
        self.get_logger().info(f'kd: "{self.kd}"')
        self.get_logger().info(f'v2')

        self.evals = Evaluations()
        self.current_score = 0

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
        self.score_pub.publish(Float32(data=self.current_score))

        #self.get_logger().info(f'min angle: "{msg.angle_min}" and max_angle: "{msg.angle_max}"')
        #self.get_logger().info(f'published angle "{ads_msg.drive.steering_angle}"and speed "{ads_msg.drive.speed}"')

    def PID_angle(self, laser_msg, current_time):
        #Creating array of desired values
        ranges_array = np.array(laser_msg.ranges)   #converts ranges array to numpy array for manipulation

        #---------------------------
        #General wall perception
        min_angle_sweep = -3*np.pi/4   #-2*np.pi/3
        max_angle_sweep =  np.pi/12 #-np.pi/6  #Gets the max angle to sweep to (min is -3pi/4)
        side_ranges_array = self.get_sub_range(min_angle_sweep, max_angle_sweep, ranges_array, laser_msg)

        #creates x and y values from each Lidar Scan point
        thetas = np.arange(min_angle_sweep, max_angle_sweep, laser_msg.angle_increment) if self.SIDE == -1 else np.arange(-1 * max_angle_sweep, -1 * min_angle_sweep, laser_msg.angle_increment)
        x_array = side_ranges_array * np.cos(thetas)
        y_array = side_ranges_array * np.sin(thetas)

        #create a LSR ?

        #limiting values to r < 3m
        #TODO: need to adjust mask for desired distance
        mask = side_ranges_array < 3
        new_x_array = x_array[mask]
        new_y_array = y_array[mask]

        coeffs = np.polyfit(new_x_array, new_y_array, 1)

        #Find the shortest distance between the line and the car
        d = np.abs(coeffs[1]) / np.sqrt(1 + (coeffs[0]**2))
        #---------------------------

        #Front wall perception (inner turn)
        min_angle_front = -np.pi/12
        max_angle_front =  np.pi/12 #Gets the max front angle
        front_ranges_array = self.get_sub_range(min_angle_front, max_angle_front, ranges_array, laser_msg)
        front_close_ratio = sum(front_ranges_array < 2)/len(front_ranges_array) #What kind of masked perception gain
        #TODO: need to adjust mask for desired distance

        #Leading wall perception (outer turn)
        min_angle_wall_lead = -np.pi/2
        max_angle_wall_lead =  -np.pi/6
        wall_lead_ranges_array = self.get_sub_range(min_angle_wall_lead, max_angle_wall_lead, ranges_array, laser_msg)
        wall_lead_far_ratio = sum(wall_lead_ranges_array > 3)/len(wall_lead_ranges_array) #What kind of masked perception drop
        #TODO: need to adjust mask for desired distance

        intensity = 0
        if front_close_ratio > 0.5:
            # inner turn, towards positive error (right wall following)
            intensity = self.DESIRED_DISTANCE*0.9
        elif wall_lead_far_ratio > 0.5:
            # outer turn, towards negative error (right wall following)
            intensity = -self.DESIRED_DISTANCE*0.9
        else:
            self.get_logger().info(f'Not turning')

        #old implementation for scaling
        #distance to a wall
        # d_wall = ranges_array[len(ranges_array)//2]
        # if d_wall < 1.5:
        #     factor = d_wall/3
        #     d  = d * factor
        #---------------------------

        #update evals
        self.current_score = self.evals.update(d, self.DESIRED_DISTANCE)

        #Determine error and PID controller
        error = (self.DESIRED_DISTANCE - d + intensity) * (-1 if self.SIDE == 1 else 1)

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
    
    def get_sub_range(self, min_angle, max_angle, range, laser_msg):
        number_of_elements_sweeped = int((max_angle - min_angle) / laser_msg.angle_increment) + 1   #determines the number of values within that sweep given the angle increment
        first_index_sweep = int(np.abs(laser_msg.angle_min - min_angle) / laser_msg.angle_increment)
        last_index_sweep = first_index_sweep + number_of_elements_sweeped
        return range[:last_index_sweep] if self.SIDE == -1 else range[-last_index_sweep:]     #Creates an array of just the desired values from that side
    
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
    