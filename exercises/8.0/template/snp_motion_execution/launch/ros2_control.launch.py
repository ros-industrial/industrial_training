from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
import yaml


parameters = [
    {'name': 'robot_description_file', 'description': 'Path to the URDF/xacro file',                  'default': ''},
    {'name': 'controllers_file',       'description': 'Path to the ros2_control configuration file',  'default': ''},
]


def declare_launch_arguments():
    return [DeclareLaunchArgument(entry['name'], description=entry['description'], default_value=entry['default']) for entry in parameters]


def generate_launch_description():
    return LaunchDescription(declare_launch_arguments() + [OpaqueFunction(function=launch)])


def launch(context, *args, **kwargs):
    # Get URDF via xacro
    robot_description_content = Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", LaunchConfiguration('robot_description_file'),])
    robot_description = {"robot_description": robot_description_content}

    controllers_file = LaunchConfiguration('controllers_file')

    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controllers_file],
            output="both",
        )
    ]

    # Load the controllers YAML file
    with open(controllers_file.perform(context), 'r') as f:
        controllers = yaml.safe_load(f)

    for key, val in controllers['controller_manager']['ros__parameters'].items():
        if type(val) is dict and type(val['type']) is str:
            nodes.append(
                Node(
                    package="controller_manager",
                    executable="spawner.py",
                    arguments=[key, "-c", "/controller_manager"],
                )
            )

    return nodes
