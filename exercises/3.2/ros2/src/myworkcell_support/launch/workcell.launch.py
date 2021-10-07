import launch
import launch_ros

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            name='fake_ar_publisher_node',
            package='fake_ar_publisher',
            executable='fake_ar_publisher_node',
            output='screen',
        ),
        launch_ros.actions.Node(
            name='vision_node',
            package='myworkcell_core',
            executable='vision_node',
            output='screen',
        ),
        launch_ros.actions.Node(
            name='myworkcell_node',
            package='myworkcell_core',
            executable='myworkcell_node',
            output='screen',
            parameters=[{'base_frame': 'world'}],
        )
    ])
