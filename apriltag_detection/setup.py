from setuptools import setup
import os
from glob import glob

package_name = 'apriltag_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 AprilTag detection package using OpenCV and pupil_apriltags',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'apriltag_detector = apriltag_detection.apriltag_detector:main',
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),  # Install package.xml explicitly
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Marker file
    ],
)

