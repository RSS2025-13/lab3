#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Point
import math


class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")

        self.declare_parameter("buffer_distance", 0.05)
        self.declare_parameter("stopping_distance_ratio", 0.6)

        self.BUFF_DIST = self.get_parameter("buffer_distance").get_parameter_value().double_value
        self.STOP_RATIO = self.get_parameter("stopping_distance_ratio").get_parameter_value().double_value

        # Subscribe to laser scan and drive topics
        self.scan_topic = "/scan"
        self.drive_topic = "/vesc/low_level/input/safety"  # Updated drive topic
        
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.current_speed = 0.0
        self.current_steering_angle = 0.0

        self.drive_sub = self.create_subscription(AckermannDriveStamped, "vesc/low_level/ackermann_cmd", self.drive_command_callback, 10)

        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)

        
        self.get_logger().info("Safety Controller 3 initialized")

    def drive_command_callback(self, msg):
        self.current_speed = msg.drive.speed
        self.current_steering_angle = msg.drive.steering_angle

    def scan_callback(self, scan_msg):
        # Convert ranges to numpy array for easier processing
        ranges = np.array(scan_msg.ranges)
        
        # Create angles array with exactly the same length as ranges
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        # Calculate x,y coordinates of each point
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Car dimensions with safety barrier
        car_length = 0.55 + self.BUFF_DIST*2  # Adding barrier on front and back
        car_width = 0.28 + self.BUFF_DIST*2   # Adding barrier on both sides
        lidar_to_front = 0.15 + self.BUFF_DIST  # Adding barrier to front

        # Check points that could collide with car
        # Points within rectangular area of car
        in_car_box = (np.abs(y) < car_width/2) & (x > -car_length/2 + lidar_to_front) & (x < lidar_to_front)
        
        if np.any(in_car_box):
            # If any point is within car box, emergency stop
            emergency_msg = AckermannDriveStamped()
            emergency_msg.drive.speed = 0.0
            emergency_msg.header.stamp = self.get_clock().now().to_msg()
            emergency_msg.header.frame_id = "base_link"
            emergency_msg.drive.acceleration = -2.0
            self.drive_pub.publish(emergency_msg)
            min_dist = np.min(ranges[in_car_box])
            self.get_logger().warn(f"Emergency stop - obstacle in car boundary! Distance: {min_dist:.2f}m")
            return

        # For points in front of the car's front barrier, use stopping distance ratio
        front_points = (np.abs(y) < car_width/2) & (x > lidar_to_front)
        if np.any(front_points):
            min_front_dist = np.min(ranges[front_points])
            stopping_distance = abs(self.current_speed * self.STOP_RATIO)
            
            if min_front_dist < stopping_distance:
                emergency_msg = AckermannDriveStamped()
                emergency_msg.drive.speed = 0.0
                emergency_msg.header.stamp = self.get_clock().now().to_msg()
                emergency_msg.header.frame_id = "base_link"
                emergency_msg.drive.acceleration = -2.0

                self.drive_pub.publish(emergency_msg)
                self.get_logger().warn(f"Emergency stop - obstacle within stopping distance! Distance: {min_front_dist:.2f}m, Required: {stopping_distance:.2f}m")
                return
    
    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!
        
        This is used by the test cases to modify the parameters during testing. 
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'buffer_distance':
                self.BUFF_DIST = param.value
                self.get_logger().info(f"Updated buffer distance to {self.BUFF_DIST}")
            if param.name == 'stopping_distance_ratio':
                self.STOP_RATIO = param.value
                self.get_logger().info(f"Updated stopping distance ratio to {self.STOP_RATIO}")
        return SetParametersResult(successful=True)
    
def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()