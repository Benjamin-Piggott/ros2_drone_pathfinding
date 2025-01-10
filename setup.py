from setuptools import setup
import os
from glob import glob

package_name = 'ros2_drone_pathfinding'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Benjamin Piggott',
    maintainer_email='ben.piggott02@gmail.com',
    description='ROS 2 implementation of drone pathfinding system with websocket visualisation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pathfinding_server = ros2_drone_pathfinding.pathfinding_server:main',
            'motor_coordinator = ros2_drone_pathfinding.motor_coordinator:main',
            'visualisation_bridge = ros2_drone_pathfinding.visualisation_bridge:main',
            'motor_controller_fl = ros2_drone_pathfinding.motor_controllers.front_left:main',
            'motor_controller_fr = ros2_drone_pathfinding.motor_controllers.front_right:main',
            'motor_controller_rl = ros2_drone_pathfinding.motor_controllers.rear_left:main',
            'motor_controller_rr = ros2_drone_pathfinding.motor_controllers.rear_right:main',
            'position_sensor = ros2_drone_pathfinding.sensors.position_sensor:main',
            'obstacle_detection = ros2_drone_pathfinding.sensors.obstacle_detection:main',
        ],
    },
)
