from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lesson_perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[
                {"cloud_topic": "/kinect/depth_registered/points"},
                {"world_frame": "world_frame"},
                {"camera_frame": "kinect_link"},
            ]
        )
     ])
