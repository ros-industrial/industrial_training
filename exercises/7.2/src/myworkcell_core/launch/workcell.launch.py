import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

def generate_launch_description():
    vision_node = launch_ros.actions.Node(
                node_name='vision_node',
                package='myworkcell_core',
                node_executable='vision_node',
                output='screen',
                arguments = [],
                parameters=[{'frame_id': 'camera_frame', 'pose_vals':[-0.6, 0.2, 0.5, 0.0, 0.0, 0.0]}])
    
    myworkcell_node = launch_ros.actions.Node(
                node_name='myworkcell_node',
                package='myworkcell_core',
                node_executable='myworkcell_node',
                output='screen',
                arguments = [],
                parameters=[{'base_frame': 'world'}])
    
    return launch.LaunchDescription([vision_node, myworkcell_node])
