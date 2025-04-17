from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gps_follower = Node(
        package='uav_control',
        executable='gps_follower',
        name='gps_follower',
        output='screen'
    )

    flight_controller = Node(
        package='uav_control',
        executable='flight_controller',
        name='flight_controller',
        output='screen'
    )

    return LaunchDescription([
        gps_follower,
        flight_controller
    ])
