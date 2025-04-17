from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'uav_following'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # ✅ Automatically detect Python packages
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package for UAV GPS following and PBVS landing',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gps_follower = uav_following.gps_follower:main',
            'pbvs_landing = uav_following.pbvs_landing:main',
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),  # ✅ Install package.xml
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # ✅ Marker file
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # ✅ Install launch files
    ],
)
