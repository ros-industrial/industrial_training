import os
import sys
import subprocess
import yaml
import launch.actions
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from industrial_moveit2_utils import MoveItConfigHelper

def load_file(package_name, file_path):
    absolute_file_path = ''
    if package_name:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
    else:
        absolute_file_path = file_path
    
    if not os.path.isfile(absolute_file_path):
        print('File "%s" was not found'%(absolute_file_path))
        sys.exit(-1)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    absolute_file_path = ''
    if package_name:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
    else:
        absolute_file_path = file_path
    
    if not os.path.isfile(absolute_file_path):
        print('File "%s" was not found'%(absolute_file_path))
        sys.exit(0)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context, *args, **kwargs):
    
    # moveit configuration
    moveit_config_pkg_name = launch.substitutions.LaunchConfiguration('moveit2_config_pkg').perform(context)
    xacro_file_path = launch.substitutions.LaunchConfiguration('xacro_file').perform(context)
    srdf_file_path = launch.substitutions.LaunchConfiguration('srdf_file').perform(context)
    ur_type = launch.substitutions.LaunchConfiguration('ur_type').perform(context)
        
    # moveit parameters    
    xacro_args = {'ur_type' : ur_type}
    moveit_config_parameters = MoveItConfigHelper(moveit_config_pkg_name, xacro_file_path, xacro_args, srdf_file_path).load_config()
    moveitcpp_parameters = moveit_config_parameters.moveit_cpp_parameters_list    
    
    # pick and place configuration
    pick_and_place_parameters = load_yaml('pick_and_place_application', 'config/pick_and_place_parameters.yaml')    
    
    # MoveItCpp demo executable
    pick_and_place_node = Node(
        name="pick_and_place_node",
        package="pick_and_place_application",
        #prefix='xterm -e gdb -ex run --args',
        executable="pick_and_place_node",
        arguments = '--ros-args --log-level info'.split(' '),
        parameters= moveitcpp_parameters + [pick_and_place_parameters],
    )    
    
    # Gathering launch entities
    launch_entities = [pick_and_place_node]
    
    return launch_entities

def generate_launch_description():
    
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument('moveit2_config_pkg',
                                              default_value = ['ur5_workcell_moveit2_config']),
        launch.actions.DeclareLaunchArgument('xacro_file', default_value = [ os.path.join(get_package_share_directory('robot_workcell_support'),
                                                                                          'urdf','ur5_workcell.xacro')]),
        launch.actions.DeclareLaunchArgument('srdf_file', default_value = [os.path.join(get_package_share_directory('ur5_workcell_moveit2_config'),
                                                                                          'config','ur5_workcell.srdf')]),
        launch.actions.DeclareLaunchArgument('ur_type',
                                             default_value = 'ur5',
                                             description = 'Type/series of used UR robot. Choices=[\'ur3\', \'ur3e\', \'ur5\', \'ur5e\', \'ur10\', \'ur10e\', \'ur16e\']'),
        OpaqueFunction(function = launch_setup)
        ])
