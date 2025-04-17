from setuptools import setup
import os
from glob import glob

package_name = 'uav_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clara',
    maintainer_email='your_email@example.com',
    description='ROS 2 package for UAV GPS following and external flight control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_follower = uav_control.gps_follower:main',
            'flight_controller = uav_control.flight_controller:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Standard ament package index
        ('share/' + package_name, ['package.xml']),  # Ensures package.xml is installed
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # Installs all launch files
    ],
)
