import glob
import os
from setuptools import find_packages
from setuptools import setup

package_name = 'robot_wall_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/'+package_name, ['package.xml', "robot_wall_follower/params.yaml"]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/robot_wall_follower/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
        ('share/robot_wall_follower/launch', glob.glob(os.path.join('launch', '*launch.py')))],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastian',
    maintainer_email='sebastianag2002@gmail.com',
    description='Wall Follower ROS2 Package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_wall_follower = robot_wall_follower.robot_wall_follower:main',
	        'viz_example = robot_wall_follower.viz_example:main',
        	'test_wall_follower = robot_wall_follower.test_wall_follower:main',
            'safety_controller = robot_wall_follower.safety_controller:main',
            'sc_exp1 = robot_wall_follower.sc_exp1:main',
            'sc_exp2 = robot_wall_follower.sc_exp2:main',
            'braketest = robot_wall_follower.braketest:main',
        ],
    },
)
