from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('safety_controller')
    
    # Path to the config file
    config_file = os.path.join(pkg_dir, 'config', 'safety_controller_params.yaml')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='../config/safety_controller_params.yaml'
        ),
        
        # Nodes to launch
        Node(
            package='safety_controller',
            executable='safety_controller_node',
            name='safety_controller',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        )
    ])



# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():
#     return LaunchDescription([
#         # Launch arguments
#         DeclareLaunchArgument(
#             'scan_topic',
#             default_value='scan',
#             description='Topic name for the laser scan data'
#         ),
#         DeclareLaunchArgument(
#             'drive_topic',
#             default_value='drive',
#             description='Topic name for the drive commands'
#         ),
#         DeclareLaunchArgument(
#             'min_safe_distance',
#             default_value='0.5',
#             description='Minimum safe distance in meters'
#         ),
#         DeclareLaunchArgument(
#             'danger_zone_distance',
#             default_value='1.0',
#             description='Distance to start slowing down in meters'
#         ),
#         DeclareLaunchArgument(
#             'safety_angle_range',
#             default_value='1.0',
#             description='Radians to check in front of the vehicle'
#         ),
#         DeclareLaunchArgument(
#             'max_speed',
#             default_value='2.0',
#             description='Maximum allowed speed'
#         ),
        
#         # Nodes to launch
#         Node(
#             package='safety_controller',
#             executable='safety_controller_node',
#             name='safety_controller',
#             output='screen',
#             parameters=[{
#                 'scan_topic': LaunchConfiguration('scan_topic'),
#                 'drive_topic': LaunchConfiguration('drive_topic'),
#                 'min_safe_distance': LaunchConfiguration('min_safe_distance'),
#                 'danger_zone_distance': LaunchConfiguration('danger_zone_distance'),
#                 'safety_angle_range': LaunchConfiguration('safety_angle_range'),
#                 'max_speed': LaunchConfiguration('max_speed'),
#             }]
#         )
#     ])