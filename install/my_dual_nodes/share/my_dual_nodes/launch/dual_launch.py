import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='my_dual_nodes',
            executable='script1',
            name='node1'
        ),
        launch_ros.actions.Node(
            package='my_dual_nodes',
            executable='script2',
            name='node2'
        ),
    ])
