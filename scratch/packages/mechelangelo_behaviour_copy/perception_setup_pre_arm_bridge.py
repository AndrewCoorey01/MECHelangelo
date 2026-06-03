# SCRATCH COPY — setup.py as of 2026-06-03
# Saved before arm_pose_bridge was registered as a console_scripts entry point.
# The arm_pose_bridge node did not exist in this version.

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mechelangelo_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andy',
    maintainer_email='andy@todo.todo',
    description='MECHelangelo human pose tracking and mimicry perception package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Main perception node — supports --mode tracking / mimicry / full
            'pose_tracking_human_ros = mechelangelo_perception.pose_tracking_human_ros:main',
            # MQTT → ROS bridge for arm angles (used on physical robot)
            'mqtt_bridge = mechelangelo_perception.mqtt_bridge:main',
        ],
    },
)
