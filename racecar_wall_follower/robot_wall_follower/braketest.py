#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Point
import math

class BrakeTest(Node):

    def __init__(self):
        super().__init__("braketest")

        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("drive_topic", "/vesc/high_level/input/nav_0")

        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = 0.0
        msg.drive.speed = self.VELOCITY
        self.drive_publisher.publish(msg)
    
    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!
        
        This is used by the test cases to modify the parameters during testing. 
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
        return SetParametersResult(successful=True)
    
def main(args=None):
    rclpy.init(args=args)

    node = BrakeTest()

    rclpy.spin(node)

    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
