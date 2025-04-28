from setuptools import setup
import os
from glob import glob

package_name = 'dual_uav_pid'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package running two nodes',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'script1 = dual_uav_pid.script1:main',
            'script2 = dual_uav_pid.script2:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # ✅ Fix: Install Launch Files
    ],
)
