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
import xacro
from xacro import load_yaml

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
 
 
class MoveitConfigParameters:

    def __init__(self):
        
        self.robot_description = None
        self.moveit_cpp_parameters_list = None
        self.ros_controller_manager_yaml_file = None
        self.ros_controllers_list = None   
        
class MoveItConfigLoader:
    
    def __init__(self, pkg_name, xacro_file_path, srdf_file):
        
        self.moveit_pkg_name = pkg_name
        self.xacro_file_path = xacro_file_path
        self.srdf_file = srdf_file        
        self.urdf_prefix = '""'
        self.config_dir = 'config'
        self.moveit_cpp_yaml_file = 'moveit_cpp.yaml'
        self.kinematics_file = 'kinematics.yaml'
        self.controllers_file = 'controllers.yaml'
        self.planning_pipeline_file = 'planning_pipeline.yaml'
        self.ompl_planning_file = 'ompl_planning.yaml'
        self.joint_limits_file = 'joint_limits.yaml'
        self.ros_controllers_yaml_file = 'ros_controllers.yaml'
        self.sensors_yaml_file = 'sensors.yaml'

    def load_config(self):
        
        # FILES RELATIVE PATHS
        MOVEIT_CPP_YAML_PATH = os.path.join(self.config_dir, self.moveit_cpp_yaml_file )
        KINEMATICS_FILE_PATH = os.path.join(self.config_dir, self.kinematics_file)
        MOVEIT_CONTROLLERS_FILE_PATH = os.path.join(self.config_dir, self.controllers_file)
        PLANNING_PIPELINE_FILE_PATH = os.path.join(self.config_dir, self.planning_pipeline_file)
        OMPL_PLANNING_FILE_PATH = os.path.join(self.config_dir, self.ompl_planning_file)
        JOINT_LIMITS_FILE_PATH = os.path.join(self.config_dir, self.joint_limits_file)
        ROS_CONTROLLERS_FILE_PATH = os.path.join(self.config_dir, self.ros_controllers_yaml_file)
        SENSORS_FILE_PATH = os.path.join(self.config_dir, self.sensors_yaml_file)
        
        # creating parameter dictionaries      
        urdf_file_content = xacro.process_file(self.xacro_file_path).toxml()  
        srdf_file_content = load_file('',self.srdf_file)
        
        moveit_cpp_yaml = load_yaml(self.moveit_pkg_name, MOVEIT_CPP_YAML_PATH)
        robot_description = {'robot_description': urdf_file_content}
        robot_description_semantic = {'robot_description_semantic': srdf_file_content}
        kinematics_yaml = load_yaml(self.moveit_pkg_name, KINEMATICS_FILE_PATH)
        
        moveit_controller_yaml = load_yaml(self.moveit_pkg_name, MOVEIT_CONTROLLERS_FILE_PATH)
        moveit_controllers = {
            'moveit_simple_controller_manager' : moveit_controller_yaml,
            'moveit_controller_manager' : 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
        }
        
        ompl_planning_pipeline_config = load_yaml(self.moveit_pkg_name, PLANNING_PIPELINE_FILE_PATH)
        ompl_planning_yaml = load_yaml(self.moveit_pkg_name, OMPL_PLANNING_FILE_PATH)
        ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)
        
        joint_limits_yaml = {'robot_description_planning' : load_yaml(self.moveit_pkg_name, JOINT_LIMITS_FILE_PATH)}
        
        parameters = [moveit_cpp_yaml,
                      robot_description,
                      robot_description_semantic,
                      kinematics_yaml,
                      moveit_controllers,
                      ompl_planning_pipeline_config,
                      joint_limits_yaml,
                      sensors_yaml]
        
        # parsing ros controllers configuration
        ros_controllers_yaml_path = os.path.join(get_package_share_directory(self.moveit_pkg_name),
                                                                          ROS_CONTROLLERS_FILE_PATH)
        ros_controllers_yaml = load_yaml('', ros_controllers_yaml_path)
        ros_controllers_list = []
        for controller_name, params in ros_controllers_yaml.items():
            if controller_name == 'controller_manager':
                continue
            print('Found ROS controller: %s' % (controller_name))
            ros_controllers_list.append(controller_name)
                   
        # create moveit config
        moveit_config_params = MoveitConfigParameters()
        moveit_config_params.robot_description = robot_description
        moveit_config_params.moveit_cpp_parameters_list = parameters
        moveit_config_params.ros_controller_manager_yaml_file = ros_controllers_yaml_path
        moveit_config_params.ros_controllers_list = ros_controllers_list
        
        return moveit_config_params  

def launch_setup(context, *args, **kwargs):
    
    # moveit configuration
    moveit_config_pkg_name = launch.substitutions.LaunchConfiguration('moveit2_config_pkg').perform(context)
    xacro_file_path = launch.substitutions.LaunchConfiguration('xacro_file').perform(context)
    srdf_file_path = launch.substitutions.LaunchConfiguration('srdf_file').perform(context)
    moveit_config_parameters = MoveItConfigLoader(moveit_config_pkg_name, xacro_file_path, srdf_file_path).load_config()
    moveitcpp_parameters = moveit_config_parameters.moveit_cpp_parameters_list
    
    # pick and place configuration
    pick_and_place_parameters = load_yaml('pick_and_place_application', 'config/ur5/pick_and_place_parameters.yaml')
    
    # MoveItCpp demo executable
    pick_and_place_node = Node(
        name="pick_and_place_node",
        package="pick_and_place_application",
        # prefix='xterm -e gdb --args',
        executable="pick_and_place_node",
        output="screen",
        parameters= moveitcpp_parameters + [pick_and_place_parameters],
    )
    
    # Trajectory controllers
    robot_description = moveit_config_parameters.robot_description
    ros2_controllers_yaml = moveit_config_parameters.ros_controller_manager_yaml_file
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters= [robot_description, ros2_controllers_yaml],  #[controller_ros_parameters],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    
    # Load controllers
    load_controllers_processes = []
    for controller in moveit_config_parameters.ros_controllers_list:
        load_controllers_processes += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]
        
    # grasp server
    fake_grasp_server = Node(
        package = 'pick_and_place_application',
        executable = 'fake_grasp_server_node',
        name= 'fake_grasp_server_node',
        output = 'screen'
        )
    
    return [pick_and_place_node, 
            fake_grasp_server,
            ros2_control_node] + load_controllers_processes

def generate_launch_description():
    
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument('moveit2_config_pkg',
                                              default_value = ['ur5_workcell_moveit2_config']),
        launch.actions.DeclareLaunchArgument('xacro_file', default_value = [ os.path.join(get_package_share_directory('robot_workcell_support'),
                                                                                          'urdf','ur5_workcell.xacro')]),
        launch.actions.DeclareLaunchArgument('srdf_file', default_value = [os.path.join(get_package_share_directory('ur5_workcell_moveit2_config'),
                                                                                          'config','ur5_workcell.srdf')]),
        OpaqueFunction(function = launch_setup)
        ])
