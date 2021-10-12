import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

def generate_launch_description():

    myworkcell_node = launch_ros.actions.Node(
                node_name='myworkcell_node',
                package='myworkcell_core',
                node_executable='myworkcell_node',
                output='screen',
                arguments = [],
                parameters=[{'base_frame': 'world'}])
    
    return launch.LaunchDescription([myworkcell_node])
