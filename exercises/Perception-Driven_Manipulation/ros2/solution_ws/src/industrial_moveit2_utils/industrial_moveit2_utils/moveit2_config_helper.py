import os
import sys
import yaml
import xacro
#from xacro import load_yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

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
 
 
class MoveItConfigParameters:

    def __init__(self):
        
        self.robot_description = None
        self.moveit_cpp_parameters_list = None
        self.move_group_parameters_list = None
        self.rviz_parameters_list = None
        self.ros_controller_manager_yaml_file = None
        self.ros_controllers_list = None  
        self.rviz_config_file =None 
        
class MoveItConfigHelper:
    
    def __init__(self, pkg_name, xacro_file_path, xacro_arguments, srdf_file):
        
        self.moveit_pkg_name = pkg_name
        self.xacro_file_path = xacro_file_path
        self.xacro_arguments = xacro_arguments
        self.srdf_file = srdf_file  
              
        self.config_dir = 'config'
        self.moveit_cpp_yaml_file = 'moveit_cpp.yaml'
        self.kinematics_file = 'kinematics.yaml'        
        self.moveit2_controllers_file = 'controllers.yaml'
        self.planning_pipeline_file = 'planning_pipeline.yaml'
        self.ompl_planning_file = 'ompl_planning.yaml'
        self.joint_limits_file = 'joint_limits.yaml'
        self.ros_controllers_yaml_file = 'ros_controllers.yaml'
        self.sensors_yaml_file = 'sensors.yaml'
        self.planning_scene_monitor_file = 'planning_scene_monitor.yaml'
        self.trajectory_execution_file = 'trajectory_execution.yaml'
        self.rviz_config_file = 'moveit.rviz'

    def load_config(self):
        
        # FILES RELATIVE PATHS
        MOVEIT_CPP_YAML_PATH = os.path.join(self.config_dir, self.moveit_cpp_yaml_file )
        KINEMATICS_FILE_PATH = os.path.join(self.config_dir, self.kinematics_file)
        MOVEIT_CONTROLLERS_FILE_PATH = os.path.join(self.config_dir, self.moveit2_controllers_file)
        PLANNING_PIPELINE_FILE_PATH = os.path.join(self.config_dir, self.planning_pipeline_file)
        OMPL_PLANNING_FILE_PATH = os.path.join(self.config_dir, self.ompl_planning_file)
        JOINT_LIMITS_FILE_PATH = os.path.join(self.config_dir, self.joint_limits_file)
        ROS_CONTROLLERS_FILE_PATH = os.path.join(self.config_dir, self.ros_controllers_yaml_file)
        SENSORS_FILE_PATH = os.path.join(self.config_dir, self.sensors_yaml_file)
        PLANNING_SCENE_MONITOR_FILE_PATH = os.path.join(self.config_dir, self.planning_scene_monitor_file)
        TRAJECTORY_EXECUTION_FILE_PATH = os.path.join(self.config_dir, self.trajectory_execution_file)
        RVIZ_FILE_PATH = os.path.join(self.config_dir, self.rviz_config_file )        
        
        # creating parameter dictionaries  
        urdf_file_content = xacro.process_file(self.xacro_file_path, mappings = self.xacro_arguments).toxml()  
        srdf_file_content = load_file('',self.srdf_file)
        
        moveit_cpp_yaml = load_yaml(self.moveit_pkg_name, MOVEIT_CPP_YAML_PATH)
        robot_description = {'robot_description': urdf_file_content}
        robot_description_semantic = {'robot_description_semantic': srdf_file_content}
        kinematics_yaml = load_yaml(self.moveit_pkg_name, KINEMATICS_FILE_PATH)
        
        # moveit controllers
        moveit_controller_yaml = load_yaml(self.moveit_pkg_name, MOVEIT_CONTROLLERS_FILE_PATH)
        moveit_controllers = {
            'moveit_simple_controller_manager' : moveit_controller_yaml,
            'moveit_controller_manager' : 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
        }
            
        ompl_planning_pipeline_config = load_yaml(self.moveit_pkg_name, PLANNING_PIPELINE_FILE_PATH)
        ompl_planning_yaml = load_yaml(self.moveit_pkg_name, OMPL_PLANNING_FILE_PATH)
        ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)
        
        joint_limits_yaml = {'robot_description_planning' : load_yaml(self.moveit_pkg_name, JOINT_LIMITS_FILE_PATH)}
        
        sensors_yaml = load_yaml(self.moveit_pkg_name, SENSORS_FILE_PATH)
        
        planning_scene_monitor_yaml = load_yaml(self.moveit_pkg_name, PLANNING_SCENE_MONITOR_FILE_PATH)
        
        trajectory_execution_yaml = load_yaml(self.moveit_pkg_name, TRAJECTORY_EXECUTION_FILE_PATH)
        
        moveitcpp_parameters = [moveit_cpp_yaml,
                      robot_description,
                      robot_description_semantic,
                      kinematics_yaml,
                      moveit_controllers,
                      ompl_planning_pipeline_config,
                      joint_limits_yaml,
                      sensors_yaml]
        
        move_group_parameters = [
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution_yaml,
            moveit_controllers,
            planning_scene_monitor_yaml,
            joint_limits_yaml,
            sensors_yaml]
        
        rviz_parameters = [
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            joint_limits_yaml,
        ]
        
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
        moveit_config_params = MoveItConfigParameters()
        moveit_config_params.robot_description = robot_description
        moveit_config_params.moveit_cpp_parameters_list = moveitcpp_parameters
        moveit_config_params.move_group_parameters_list = move_group_parameters
        moveit_config_params.rviz_parameters_list = rviz_parameters
        moveit_config_params.ros_controller_manager_yaml_file = ros_controllers_yaml_path
        moveit_config_params.ros_controllers_list = ros_controllers_list
        moveit_config_params.rviz_config_file = os.path.join(get_package_share_directory(self.moveit_pkg_name),
                                                                                          RVIZ_FILE_PATH)
        
        return moveit_config_params
    
    def create_executables(self, use_rviz = True):     
           
        moveit_config_parameters = self.load_config() 
        robot_description = moveit_config_parameters.robot_description
        
        # move group node
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="log",
            parameters = moveit_config_parameters.move_group_parameters_list,
        )
        
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        )   
        
        # Robot Trajectory controllers
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
                    cmd=["ros2 run controller_manager spawner {}".format(controller)],
                    shell=True,
                    output="screen",
                )
            ]
            
        # Gathering launch entities
        launch_entities = [
            move_group_node, 
            robot_state_publisher,
            ros2_control_node
            ]
        
        launch_entities += load_controllers_processes
        
        # optionally adding rviz
        if use_rviz:
            rviz_node = Node(
                package = 'rviz2',
                name = 'moveit_rviz2',
                executable = 'rviz2',
                output = 'log',
                arguments = ['-d', moveit_config_parameters.rviz_config_file],
                parameters = moveit_config_parameters.rviz_parameters_list)  
            
            launch_entities.append(rviz_node)
        
    
        return launch_entities