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
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 implementation of drone pathfinding system with websocket visualisation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pathfinding_server = ros2_drone_pathfinding.pathfinding_server:main',
            'motor_controller = ros2_drone_pathfinding.motor_controller:main',
            'visualisation_bridge = ros2_drone_pathfinding.visualisation_bridge:main',
        ],
    },
)
