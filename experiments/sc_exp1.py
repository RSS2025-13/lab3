#!/usr/bin/env python3
# TURNING RADIUS + BOX BASED APPROACH


import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import SetParametersResult

'''
Sim: ros2 launch racecar_simulator simulate.launch.xml
To launch: ros2 launch safety_controller safety_controller.launch.py
To test driving commands: ros2 topic pub --once /drive ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 2.0, steering_angle: 0.0}}"

The safety controller operates as follows:

Subscribes to LIDAR scan data and drive commands
Monitors for obstacles within the safety_angle_range
If an obstacle is detected:
- Stops the car if the obstacle is closer than min_safe_distance
- Gradually slows the car if the obstacle is within danger_zone_distance

Forwards unmodified commands when no obstacles are detected
Only affects forward motion, allowing for full-speed reversing
'''

class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/low_level/input/safety")
        self.declare_parameter("nav_cmd_topic", "/vesc/low_level/ackermann_cmd")
        self.declare_parameter("min_safe_distance", 0.1)  # minimum safe distance in meters
        self.declare_parameter("danger_zone_distance", 0.2)  # distance to start slowing down
        self.declare_parameter("safety_angle_range", 1.0)  # radians to check in front
        self.declare_parameter("max_speed", 2.0)  # maximum allowed speed

        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.NAV_CMD_TOPIC = self.get_parameter('nav_cmd_topic').get_parameter_value().string_value
        self.MIN_SAFE_DISTANCE = self.get_parameter('min_safe_distance').get_parameter_value().double_value
        self.DANGER_ZONE_DISTANCE = self.get_parameter('danger_zone_distance').get_parameter_value().double_value
        self.SAFETY_ANGLE_RANGE = self.get_parameter('safety_angle_range').get_parameter_value().double_value
        self.MAX_SPEED = self.get_parameter('max_speed').get_parameter_value().double_value


        self.scan_subscriber = self.create_subscription(
            LaserScan, 
            self.SCAN_TOPIC, 
            self.scan_callback, 
            10
        )

        self.nav_cmd_subscriber = self.create_subscription(
            AckermannDriveStamped, 
            self.NAV_CMD_TOPIC, 
            self.nav_cmd_callback, 
            10
        )
    
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, 
            self.DRIVE_TOPIC, 
            10
        )
        
        self.is_safe = True
        self.latest_nav_cmd = None
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.CAR_LENGTH = 0.55
        self.CAR_WIDTH = 0.38
        self.DIST_TO_BUMPER = 0.15

        self.get_logger().info("Safety Controller initialized")

    def stop_cmd(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        if self.latest_nav_cmd:
            msg.drive.steering_angle = self.latest_nav_cmd.drive.steering_angle
        self.drive_publisher.publish(msg)

    def nav_cmd_callback(self, msg):
        # Store the latest navigation command
        self.latest_nav_cmd = msg

    def scan_callback(self, msg):
        
        if self.latest_nav_cmd is not None:
            speed = self.latest_nav_cmd.drive.speed
            steering_angle = self.latest_nav_cmd.drive.steering_angle
        else:
            speed = 1.0
            steering_angle = 0.0

        # EXPERIMENTATION NEED TO TUNE!!
        stop_distance = (0.5 * speed + 0.2) ** 2

        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        front_distances = np.array(msg.ranges)
        x = np.cos(angles) * front_distances
        y = np.sin(angles) * front_distances

        #-----------------------------------------------------------------------------------------------------------------------
        # APPROACH 1 (for steering angle = 0): create rectangle at the car front and check if any points within it - if so, stop

        y_lower_bound = -0.5 * self.CAR_WIDTH
        y_upper_bound = 0.5 * self.CAR_WIDTH
        x_lower_bound = self.DIST_TO_BUMPER
        x_upper_bound = stop_distance + self.DIST_TO_BUMPER

        y_in_rectangle = (y >= y_lower_bound) & (y <= y_upper_bound)
        x_check = x[y_in_rectangle]
        in_rectangle = (x_check >= x_lower_bound) & (x_check <= x_upper_bound)

        if np.any(in_rectangle) and steering_angle == 0.0:
            self.stop_cmd()
            return
        #------------------------------------------------------------------------------------------------------------------------
        

        #-------------------------------------------------------------------------------------------------------------------------
        # APPROACH 2 (for non-zero steering angle)

        turning_sign = np.sign(steering_angle)
        turning_radius = np.abs(self.CAR_LENGTH / np.sin(steering_angle))
        inner_wheel_radius = turning_radius - 0.5 * self.CAR_WIDTH
        outer_wheel_radius = turning_radius + 0.5 * self.CAR_WIDTH

        inner_circle = turning_sign * inner_wheel_radius - turning_sign * np.sqrt(inner_wheel_radius**2 - x**2)
        outer_circle = turning_sign * outer_wheel_radius - turning_sign * np.sqrt(outer_wheel_radius**2 - x**2)

        distance_line = - turning_sign * np.tan(np.pi / 2 - stop_distance / turning_radius) * x + turning_sign * turning_radius
            
        if turning_sign == 1:
            trace_upper = inner_circle
            trace_lower = outer_circle
        else: 
            trace_upper = outer_circle
            trace_lower = inner_circle
            
        inside_curve = (y >= trace_lower) & (y <= trace_upper)
        dist_check = y[inside_curve]
        distance_line = distance_line[inside_curve]
        too_close_curve = dist_check >= distance_line

        # stop car if any points in donut
        if np.any(too_close_curve):
            self.stop_cmd()


        #--------------------------------------------------------------------------------------------------------------------------
        
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'min_safe_distance':
                self.MIN_SAFE_DISTANCE = param.value
                self.get_logger().info(f"Updated min_safe_distance to {self.MIN_SAFE_DISTANCE}")
            elif param.name == 'danger_zone_distance':
                self.DANGER_ZONE_DISTANCE = param.value
                self.get_logger().info(f"Updated danger_zone_distance to {self.DANGER_ZONE_DISTANCE}")
            elif param.name == 'safety_angle_range':
                self.SAFETY_ANGLE_RANGE = param.value
                self.get_logger().info(f"Updated safety_angle_range to {self.SAFETY_ANGLE_RANGE}")
            elif param.name == 'max_speed':
                self.MAX_SPEED = param.value
                self.get_logger().info(f"Updated max_speed to {self.MAX_SPEED}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
