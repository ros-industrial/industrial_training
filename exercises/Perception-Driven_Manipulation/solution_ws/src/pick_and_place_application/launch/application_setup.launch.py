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

def launch_setup(context, *args, **kwargs):
    
    # moveit configuration
    moveit_config_pkg_name = launch.substitutions.LaunchConfiguration('moveit2_config_pkg').perform(context)
    xacro_file_path = launch.substitutions.LaunchConfiguration('xacro_file').perform(context)
    srdf_file_path = launch.substitutions.LaunchConfiguration('srdf_file').perform(context)
    
    # parsing xacro arguments
    use_sim_robot = launch.substitutions.LaunchConfiguration('use_sim_robot').perform(context)    
    init_positions_file_path = launch.substitutions.LaunchConfiguration('initial_positions_file').perform(context)  
    
    # laoding moveit configuration   
    xacro_args = {'use_fake_hardware': use_sim_robot, 'initial_positions_file' : init_positions_file_path}
    moveit_config_helper = MoveItConfigHelper(moveit_config_pkg_name, xacro_file_path, xacro_args, srdf_file_path)
    
    # declaring list to hold all executables
    launch_entities = []
    
    # getting moveit executations
    moveit_config_parameters = moveit_config_helper.load_config()
    moveit_nodes = moveit_config_helper.create_executables(use_rviz = False)
    launch_entities += moveit_nodes
    
    # instantiating application specific nodes
    # fake ar tag transform (simulation only)
    fake_ar_tag_transform_publisher = Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        name = 'fake_ar_tag_transform_publisher',
        arguments = '-0.8 0.2 0.17 0.785 0 0 world_frame ar_tag'.split(' '),
        output = 'log')
        
    # simulated grasp server
    fake_grasp_server = Node(
        package = 'pick_and_place_application',
        executable = 'fake_grasp_server_node',
        name= 'fake_grasp_server_node',
        output = 'screen'
        )
    
    # fake recognition service
    fake_recognition_server = Node(
        package = 'pick_and_place_application',
        executable = 'fake_recognition_service.py',
        name = 'fake_recognition_service',
        output = 'screen')
    
    # fake obstacles cloud
    #<remap from="/generated_cloud" to="/kinect/depth_registered/points"/>
    cloud_yaml_file = os.path.join(get_package_share_directory('pick_and_place_application'),'config','fake_obstacles_cloud_description.yaml')
    fake_obstacles_cloud_node = Node(
        package = 'pick_and_place_application',
        executable = 'generate_cloud_node',
        name = 'fake_obstacles_cloud_node',
        remappings = [('generated_cloud' ,'objects_cloud')],
        parameters = [{'obstacles_cloud_description_file':cloud_yaml_file}],
        output = 'screen')
    
    rviz_config_file = os.path.join(get_package_share_directory('pick_and_place_application'), 'config', 'rviz_config.rviz')
    rviz_node = Node(
        package = 'rviz2',
        name = 'rviz2',
        executable = 'rviz2',
        output = 'log',
        arguments = ['-d', rviz_config_file] + '--ros-args --log-level error'.split(' '),
        parameters = moveit_config_parameters.rviz_parameters_list)     
    
    launch_entities += [fake_ar_tag_transform_publisher,
                         fake_grasp_server,
                         fake_obstacles_cloud_node,
                         fake_recognition_server,
                         rviz_node,
                         ]
    
    return launch_entities    

def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument('moveit2_config_pkg', default_value = ['ur5_workcell_moveit2_config']),
        launch.actions.DeclareLaunchArgument('xacro_file', default_value = [os.path.join(get_package_share_directory('robot_workcell_support'),
                                                                                    'urdf','ur5_workcell.xacro')]),
        launch.actions.DeclareLaunchArgument('initial_positions_file',
                                             default_value = [ os.path.join(get_package_share_directory('ur5_workcell_moveit2_config'), 'config', 'initial_positions.yaml')]),
        launch.actions.DeclareLaunchArgument('use_sim_robot', default_value = ['True']),
        launch.actions.DeclareLaunchArgument('srdf_file', default_value = [os.path.join(get_package_share_directory('ur5_workcell_moveit2_config'),
                                                                                          'config','ur5_workcell.srdf')]),
        OpaqueFunction(function = launch_setup)
        ])