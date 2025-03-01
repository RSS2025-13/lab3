#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
import math

# from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

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

        self.subscription = self.create_subscription(
            LaserScan, self.SCAN_TOPIC, self.scan_callback, 10)
        self.publisher = self.create_publisher(
            AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        
        # self.KD = 0.1
        # if self.VELOCITY == 3:
        #     self.KP = 1.0
        #     self.KD = 0.7
        # elif self.VELOCITY == 2: 
        #     self.KP = 0.7
        #     self.KD = 1.5
        # # elif self.VELOCITY > 1:
        # #     self.KP = 0.7
        # #     self.KD = 4
        # else:
        #     self.KP = 0.4
        #     self.KD = 0.1

        self.KP = 0.5
        self.KD = 0.1
        if self.VELOCITY > 1:
            self.KP = 2.5
            self.KD = 9

        elif self.VELOCITY > 2:
            self.KP = 2
            self.KD = 9



        self.prev_error = 0.0


    def scan_callback(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        distances = np.array(msg.ranges)
        
        if self.SIDE == 1:  # Left side
            angle_mask = (angles > -np.pi/8) & (angles < np.pi/2)
        else:  # Right side
            angle_mask = (angles < 0) & (angles > -np.pi/2)

        # distance_mask = distances <= 6.0
    
        # Combine both filters with logical AND
        valid_indices = angle_mask #& distance_mask
        
        selected_angles = angles[valid_indices]
        selected_distances = distances[valid_indices]

        x_vals = selected_distances * np.cos(selected_angles)
        y_vals = selected_distances * np.sin(selected_angles)
        
        # if len(x_vals) < 5:
        #     self.get_logger().warn("Not enough valid LIDAR points to estimate the wall.")
        #     return
        
        A = np.vstack([x_vals, np.ones_like(x_vals)]).T
        m, b = np.linalg.lstsq(A, y_vals, rcond=None)[0]
        
        distance_to_wall = abs(b) / np.sqrt(1 + m**2)
        error = self.DESIRED_DISTANCE - distance_to_wall

        error += 0.55
        curvature = abs(m)
    
        # # Adjust speed based on curvature
        # speed_factor = 1.0 / (1.0 + 2.0 * curvature)  # Reduce speed as curvature increases
        # adjusted_speed = self.VELOCITY * max(1, speed_factor)

        derivative = error - self.prev_error
        steering_angle = self.KP * error + self.KD * derivative
        self.prev_error = error
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.VELOCITY
        # drive_msg.drive.speed = adjusted_speed
        drive_msg.drive.steering_angle = -self.SIDE*steering_angle
        self.publisher.publish(drive_msg)

        # wall_angle = math.atan(m)

        # KP = self.KP * (1 + 0.1 * self.VELOCITY)  # Increase with speed
        # KD = self.KD / (1 + 0.1 * self.VELOCITY)  # Reduce damping at high speed

        # derivative = error - self.prev_error
        # if self.SIDE == 1:  # Following left wall
        #     steering_angle = KP * error + KD * derivative + 0.8 * wall_angle
        # else:  # Following right wall
        #     steering_angle = KP * error + KD * derivative - 0.8 * wall_angle

        # self.prev_error = error

        # # speed = max(0.8, min(self.VELOCITY - abs(error) * 0.5, self.VELOCITY))
        # speed = self.VELOCITY

        # drive_msg = AckermannDriveStamped()
        # drive_msg.drive.speed = speed
        # drive_msg.drive.steering_angle = max(min(steering_angle, 0.5), -0.5)  # Increase steering limit
        # self.publisher.publish(drive_msg)
        
    
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
    