from setuptools import setup
import os
from glob import glob

package_name = 'uav_mission'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='UAV mission package with GPS tracking and AprilTag detection',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gps_follower = uav_mission.gps_follower:main',
            'apriltag_detector = uav_mission.apriltag_detector:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # Ensure launch files are installed
    ],
)

