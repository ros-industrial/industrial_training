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
                {"voxel_leaf_size": 0.02}, # mm
                {"x_filter_min": -2.5},    # mm
                {"x_filter_max": 2.5},     # mm
                {"y_filter_min": -2.5},    # mm
                {"y_filter_max": 2.5},     # mm
                {"z_filter_min": -2.5},    # mm
                {"z_filter_max": 2.5},     # mm
                {"plane_max_iterations": 100},
                {"plane_distance_threshold": 0.03},
                {"cluster_tolerance": 0.02},
                {"cluster_min_size": 1},
                {"cluster_max_size": 10000}
            ]
        )
     ])
