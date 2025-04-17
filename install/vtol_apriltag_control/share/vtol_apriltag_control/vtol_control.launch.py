from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='vtol_apriltag_control', executable='apriltag_detector', output='screen'),
        Node(package='vtol_apriltag_control', executable='flight_controller', output='screen'),
    ])

