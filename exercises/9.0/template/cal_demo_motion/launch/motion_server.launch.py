from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="arm", package_name="cal_demo_moveit_config"
        )
        .robot_description(file_path="config/demo_workcell.urdf.xacro")
        .moveit_cpp(
            file_path=get_package_share_directory("cal_demo_motion")
            + "/config/moveit_config.yaml"
        )
        .to_moveit_configs()
    )

    moveit = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('cal_demo_moveit_config'),
                        'launch',
                        'demo.launch.py'
                        ])
                    ])
                )

    motion_server = Node(
        name="cal_demo_motion_server",
        package="cal_demo_motion",
        executable="motion_server",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription(
        [
            moveit,
            motion_server,
        ]
    )
