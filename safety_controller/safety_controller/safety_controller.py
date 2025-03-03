#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.msg import SetParametersResult

'''
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

        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("drive_topic", "drive")
        self.declare_parameter("min_safe_distance", 0.5)  # minimum safe distance in meters
        self.declare_parameter("danger_zone_distance", 1.0)  # distance to start slowing down
        self.declare_parameter("safety_angle_range", 1.0)  # radians to check in front
        self.declare_parameter("max_speed", 2.0)  # maximum allowed speed

        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
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
        
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, 
            self.DRIVE_TOPIC, 
            10
        )
        
        self.is_safe = True
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.get_logger().info("Safety Controller initialized")

    def scan_callback(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        
        front_indices = np.where(np.abs(angles) <= self.SAFETY_ANGLE_RANGE / 2.0)
        front_distances = np.array(msg.ranges)[front_indices]
        
        valid_indices = np.isfinite(front_distances)
        front_distances = front_distances[valid_indices]
        
        # if len(front_distances) == 0:
        #     self.get_logger().warn("No valid distance readings in front of vehicle!")
        #     return
        
        min_distance = np.min(front_distances)
        
        if min_distance < self.MIN_SAFE_DISTANCE:
            # Emergency stop - obstacle is too close
            # self.get_logger().warn(f"Emergency stop! Obstacle at {min_distance:.2f}m")
            
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0.0
            stop_msg.drive.steering_angle = 0.0
            stop_msg.drive.acceleration = 0.0
            stop_msg.drive.jerk = 0.0
            
            self.drive_publisher.publish(stop_msg)
            self.is_safe = False

        elif min_distance < self.DANGER_ZONE_DISTANCE:
            # In the danger zone - calculate a safe speed proportional to distance (but keep steering as is)
            safety_factor = (min_distance - self.MIN_SAFE_DISTANCE) / (self.DANGER_ZONE_DISTANCE - self.MIN_SAFE_DISTANCE)
            
            slow_msg = AckermannDriveStamped()
            slow_msg.drive.speed = self.MAX_SPEED * safety_factor
            self.drive_publisher.publish(slow_msg)
            # self.get_logger().info(f"Slowing down! Obstacle at {min_distance:.2f}m, speed reduced to {slow_msg.drive.speed:.2f}")
            self.is_safe = True
        else:
            # INTEGRATE NORMAL WALL FOLLOWER HERE
            self.is_safe = True
        
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