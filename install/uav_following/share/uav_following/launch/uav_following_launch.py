import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='uav_following',
            executable='gps_follower',
            name='gps_follower',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='uav_following',
            executable='pbvs_landing',
            name='pbvs_landing',
            output='screen'
        ),
    ])
