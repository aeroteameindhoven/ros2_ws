from setuptools import setup
import os
from glob import glob

package_name = 'vtol_apriltag_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),  # Include launch files
    ],
    install_requires=[
        'setuptools',
        'pupil_apriltags',  # Ensure Pupil Labs AprilTag library is installed
        'opencv-python',     # Install OpenCV if not already installed
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='VTOL UAV external flight control using AprilTags and MAVLink.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'apriltag_detector = vtol_apriltag_control.apriltag_detector:main',
            'flight_controller = vtol_apriltag_control.flight_controller:main',
        ],
    },
)

