# lab3

To run new robot_wall_follower package (for sim)

1. colcon build --symlink-install
2. source install/setup.bash
3. ros2 launch robot_wall_follower launch_test_sim.launch.py
4. ros2 launch robot_wall_follower launch_test.launch.py

To test safety_controller:

1. Launch sim: ros2 launch robot_wall_follower wall_follower.launch.xml
2. To launch: ros2 launch safety_controller safety_controller.launch.py
3. To test driving commands: ros2 topic pub --once /drive ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 2.0, steering_angle: 0.0}}"
