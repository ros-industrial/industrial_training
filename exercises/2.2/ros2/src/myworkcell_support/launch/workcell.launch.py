import launch
import launch_ros

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            node_name='fake_ar_publisher_node',
            package='fake_ar_publisher',
            node_executable='fake_ar_publisher_node',
            output='screen',
            ),
        launch_ros.actions.Node(
            node_name='vision_node',
            package='myworkcell_core',
            node_executable='vision_node',
            output='screen',
            ),
    ])
