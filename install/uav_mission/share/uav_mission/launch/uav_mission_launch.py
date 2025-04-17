import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='uav_mission',
            executable='gps_follower',
            name='gps_follower_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='uav_mission',
            executable='apriltag_detector',
            name='apriltag_detector_node',
            output='screen'
        ),
    ])
