import os
import yaml
import launch
import launch_ros
from ament_index_python import get_package_share_directory

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(file_path):
    """Load a yaml file into a dictionary"""
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file


def generate_launch_description():
    xacro_file = get_package_file('myworkcell_support', 'urdf/workcell.urdf.xacro')
    urdf_file = run_xacro(xacro_file)
    srdf_file = get_package_file('myworkcell_moveit_config', 'config/myworkcell.srdf')
    kinematics_file = get_package_file('myworkcell_moveit_config', 'config/kinematics.yaml')
    ompl_config_file = get_package_file('myworkcell_moveit_config', 'config/ompl_planning.yaml')
    controllers_file = get_package_file('myworkcell_moveit_config', 'config/controllers.yaml')

    robot_description = load_file(urdf_file)
    robot_description_semantic = load_file(srdf_file)
    kinematics_config = load_yaml(kinematics_file)
    ompl_config = load_yaml(ompl_config_file)

    moveit_controllers = {
        'moveit_simple_controller_manager' : load_yaml(controllers_file),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01
    }
    planning_scene_monitor_config = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True
    }

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            #name='move_group_node',
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'robot_description_semantic': robot_description_semantic,
                    'robot_description_kinematics': kinematics_config,
                    'ompl': ompl_config,
                },
                moveit_controllers,
                trajectory_execution,
                planning_scene_monitor_config,
            ],
        ),
        launch_ros.actions.Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description}
            ]
        ),
        launch_ros.actions.Node(
            package='fake_joint_driver',
            executable='fake_joint_driver_node',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'controller_name': 'fake_joint_trajectory_controller'
                },
                get_package_file("myworkcell_moveit_config", "config/fake_controllers.yaml"),
            ],
        ),
        launch_ros.actions.Node(
            name='rviz',
            package='rviz2',
            executable='rviz2',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'robot_description_semantic': robot_description_semantic,
                    'robot_description_kinematics': kinematics_config,
                    'ompl': ompl_config,
                }
            ],
        )
    ])
