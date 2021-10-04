import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
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

def generate_launch_description():
    xacro_file = get_package_file('myworkcell_support', 'urdf/workcell.urdf.xacro')
    srdf_file = get_package_file('myworkcell_moveit_config', 'config/myworkcell.srdf')
    kinematics_file = get_package_file('myworkcell_moveit_config', 'config/kinematics.yaml')
    ompl_config_file = get_package_file('myworkcell_moveit_config', 'config/ompl_planning.yaml')
    joint_limits_file = get_package_file('myworkcell_moveit_config','config/joint_limits.yaml')
    moveit_controllers_file = get_package_file('myworkcell_moveit_config', 'config/controllers.yaml')

    robot_description = xacro.process_file(xacro_file).toprettyxml(indent='  ')
    robot_description_semantic = load_file(srdf_file)
    kinematics_config = load_yaml(kinematics_file)
    ompl_config = load_yaml(ompl_config_file)
    joint_limits_config = load_yaml(joint_limits_file)

    # Setting up MoveitCpp configuration parameters
    moveit_controllers = {
        'moveit_simple_controller_manager' : load_yaml(moveit_controllers_file),
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

    moveit_cpp_config = yaml.load("""
        planning_scene_monitor_options:
          name: "planning_scene_monitor"
          robot_description: "robot_description"
          joint_state_topic: "/joint_states"
          attached_collision_object_topic: "/moveit_cpp/planning_scene_monitor"
          publish_planning_scene_topic: "/moveit_cpp/publish_planning_scene"
          monitored_planning_scene_topic: "/moveit_cpp/monitored_planning_scene"
          wait_for_initial_state_timeout: 10.0

        planning_pipelines:
          #namespace: "moveit_cpp"  # optional, default is ~
          pipeline_names: ["ompl"]

        plan_request_params:
          planning_time: 10.0
          planning_attempts: 3
          planning_pipeline: ompl
          max_velocity_scaling_factor: 0.5
          max_acceleration_scaling_factor: 0.5

        # octomap parameters (when used)
        octomap_frame: world
        octomap_resolution: 0.01
        max_range: 5.0""")

    return LaunchDescription([
        Node(
            name='myworkcell_node',
            package='myworkcell_core',
            executable='myworkcell_node',
            output='screen',
            parameters=[
                {
                    'base_frame': 'world',
                    'robot_description': robot_description,
                    'robot_description_semantic': robot_description_semantic,
                    'robot_description_kinematics': kinematics_config,
                    'robot_description_planning' : joint_limits_config,
                    'planning_pipelines': ['ompl'],
                    'ompl': ompl_config
                },
                moveit_cpp_config,
                moveit_controllers,
                trajectory_execution,
                planning_scene_monitor_config,
            ],
        ),
        Node(
            name='fake_ar_publisher_node',
            package='fake_ar_publisher',
            executable='fake_ar_publisher_node',
            output='screen',
        ),
        Node(
            name='vision_node',
            package='myworkcell_core',
            executable='vision_node',
            output='screen',
        ),
    ])
