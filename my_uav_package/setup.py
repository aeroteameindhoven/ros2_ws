import os
from setuptools import setup

package_name = 'my_uav_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mission_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clara',
    maintainer_email='your-email@example.com',
    description='ROS2 UAV Package for GPS Following and Flight Control',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gps_follower = my_uav_package.gps_follower:main',
            'flight_controller = my_uav_package.flight_controller:main',
            'run_mission = my_uav_package.run_mission:main',
        ],
    },
)
