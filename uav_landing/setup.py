from setuptools import setup
import os
from glob import glob

package_name = 'uav_landing'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # Ensures the package is found
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='UAV speed matching and landing controller using GPS & AprilTags',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'landing_controller = uav_landing.landing_controller:main',
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),  # Ensures package.xml is installed
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # Marker file
    ],
)
