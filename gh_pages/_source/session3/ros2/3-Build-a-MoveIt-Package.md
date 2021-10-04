# Build a MoveIt! Package
>In this exercise, we will create a MoveIt! package for an industrial robot. This package creates the configuration and launch files required to use a robot with the MoveIt! Motion-Control nodes. In general, the MoveIt! package does not contain any C++ code.

> **_IMPORTANT: This exercise requires a mix of ROS1 and ROS2 environments. Be careful what ROS environment is sourced in each terminal.  The "MoveIt Setup Assistant" is not yet available in ROS2, so we run the ROS1 version.  But it outputs a ROS1-compatible package, so we need to adapt the output for ROS2._**

## Motivation
MoveIt! is a free-space motion planning framework for ROS. It’s an incredibly useful and easy-to-use tool for planning motions between two points in space without colliding with anything. Under the hood MoveIt is quite complicated, but (unlike most ROS libraries) it has a really nice GUI Wizard to get you started.

## Reference Example
[Using MoveIt with ROS-I](http://wiki.ros.org/Industrial/Tutorials/Create_a_MoveIt_Pkg_for_an_Industrial_Robot)

## Further Information and Resources
[MoveIt Setup Assistant Tutorial](http://moveit2_tutorials.picknik.ai/doc/setup_assistant/setup_assistant_tutorial.html)

## Scan-N-Plan Application: Problem Statement
In this exercise, you will generate a MoveIt package for the UR5 workcell you built in a previous step. This process will mostly involve running the MoveIt! Setup Assistant. At the end of the exercise you should have the following:

 1. A new package called `myworkcell_moveit_config`

 1. A moveit configuration with one group ("manipulator"), that consists of the kinematic chain between the UR5’s `base_link` and `tool0`.

## Scan-N-Plan Application: Guidance

### Create a Base Package using the Setup Assistant

1. Open a NEW terminal and setup a ROS1 workspace to run the MoveIt Setup Assistant.  Put copies of the required URDF packages (`ur_description`, `myworkcell_support`) inside this ROS1 workspace, to make them visible to the Setup Assistant:

   ```
   mkdir -p ~/ros1_ws/src
   cp -r ~/industrial_training/exercises/3.3/ros1/src/ur_description ~/ros1_ws/src
   cp -r ~/industrial_training/exercises/3.3/ros1/src/myworkcell_support ~/ros1_ws/src
   
   <edit ~/ros1_ws/src/myworkcell_support/package.xml & CMakeLists.txt and remove all references to myworkcell_core>
   
   cd ~/ros1_ws/
   source /opt/ros/noetic/setup.bash
   catkin build
   source ~/ros1_ws/devel/setup.bash
   ```

 1. Start the MoveIt! Setup Assistant (don't forget auto-complete with tab):

    ```
    roslaunch moveit_setup_assistant setup_assistant.launch
    ```

 1. Select "Create New MoveIt Configuration Package", select the `workcell.xacro` in ROS1 workspace's _myworkcell_support_ package, then "Load File".

 1. Work your way through the tabs on the left from the top down.

    1. Generate a self-collision matrix.
    1. Add a fixed virtual base joint. e.g.

       ```
       name = 'FixedBase' (arbitrary)
       child = 'world' (should match the URDF root link)
       parent = 'world' (reference frame used for motion planning)
       type = 'fixed'
       ```

    1. Add a planning group called `manipulator` that names the kinematic chain between `base_link` and `tool0`. Note: Follow [ROS naming guidelines/requirements](http://wiki.ros.org/ROS/Patterns/Conventions) and don't use any whitespace, anywhere.

       a. Set the kinematics solver to `KDLKinematicsPlugin`
       a. Set the default OMPL planner to `RRTConnect`

    1. Create a few named positions (e.g. "home", "allZeros", etc.) to test with motion-planning.

    1. Don't worry about adding end effectors/grippers or passive joints for this exercise.
    
    1. In the "ROS Controllers" tab, use the "Auto Add FollowJointsTrajectory" button to define a basic _FollowJointTrajectory_ controller for the entire UR5 arm.
    
    1. Skip the Simulation and 3D Perception tabs.

    1. Enter author / maintainer info.

       _Yes, it's required, but doesn't have to be valid_

    1. Generate a new package and name it `myworkcell_moveit_config`.
       * make sure to create the package inside your `ros1_ws/src` directory

## Create ROS2 Package from Setup Assistant Output

 The outcome of the Setup Assistant is a new ROS1 package that contains a large number of launch and configuration files.  We need to add a few files to customize this general-purpose package for our application and convert it to work with ROS2, where the setup assistant is not yet available.
 
 > _Note: The world of MoveIt is continually evolving in ROS2; this is currently just one way to get a ROS2 package going and is not presented as the "correct" way._

 1. Create a new empty package inside your **ROS2** workspace:

    ```
    source ~/ros2_ws/install/setup.bash
    cd ~/ros2_ws/src
    ros2 pkg create myworkcell_moveit_config --dependencies myworkcell_support
    ```

 1. Create an empty `config/` subdirectory inside the new package and copy the following files from the `config/` subdirectory of the ROS1 MoveIt config package:

    ```
    myworkcell.srdf
    joint_limits.yaml
    kinematics.yaml
    ompl_planning.yaml
    ```

 1. Open the ROS2 `config/ompl_planning.yaml` file in a text editor and add the following lines at the bottom:

    ```
    planning_plugin: 'ompl_interface/OMPLPlanner'
    request_adapters: >-
        default_planner_request_adapters/AddTimeOptimalParameterization
        default_planner_request_adapters/FixWorkspaceBounds
        default_planner_request_adapters/FixStartStateBounds
        default_planner_request_adapters/FixStartStateCollision
        default_planner_request_adapters/FixStartStatePathConstraints
    start_state_max_bounds_error: 0.1
    ```

 1. Create a new `controllers.yaml` file in the ROS2 MoveIt package's `config` directory:

    ```
    controller_names:
      - manipulator_joint_trajectory_controller

    manipulator_joint_trajectory_controller:
      action_ns: follow_joint_trajectory
      type: FollowJointTrajectory
      default: true
      joints:
        - shoulder_pan_joint
        - shoulder_lift_joint
        - elbow_joint
        - wrist_1_joint
        - wrist_2_joint
        - wrist_3_joint
    ```

    This file will configure MoveIt to use a controller for joint trajectory execution provided by `ros_control`.

 1. Create a new `ros_controllers.yaml` file in the ROS2 MoveIt package's `config` directory:

    ```
    controller_manager:
      ros__parameters:
        update_rate: 600  # Hz
        manipulator_joint_trajectory_controller:
          type: joint_trajectory_controller/JointTrajectoryController
        joint_state_controller:
          type: joint_state_controller/JointStateController

    # parameters for each controller listed under controller manager
    manipulator_joint_trajectory_controller:
      ros__parameters:
        command_interfaces:
          - position
        state_interfaces:
          - position
          - velocity
        joints:
          - elbow_joint
          - shoulder_lift_joint
          - shoulder_pan_joint
          - wrist_1_joint
          - wrist_2_joint
          - wrist_3_joint
        state_publish_rate: 100.0
        action_monitor_rate: 20.0
        allow_partial_joints_goal: false
        constraints:
          stopped_velocity_tolerance: 0.0
          goal_time: 0.0

    joint_state_controller:
      ros__parameters:
        type: joint_state_controller/JointStateController
    ```

    These parameters configure the controller nodes at startup.

 1. Create a new `launch` directory inside the ROS2 moveit package, and a new launch file, `myworkcell_planning_execution.launch.py` inside that directory.  This launch file will act as a single location to start up all components needed for both motion planning and execution.

    ```
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
        moveit_controllers_file = get_package_file('myworkcell_moveit_config', 'config/controllers.yaml')
        ros_controllers_file = get_package_file('myworkcell_moveit_config', 'config/ros_controllers.yaml')

        robot_description = load_file(urdf_file)
        robot_description_semantic = load_file(srdf_file)
        kinematics_config = load_yaml(kinematics_file)
        ompl_config = load_yaml(ompl_config_file)

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

        # MoveIt node
        move_group_node = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'robot_description_semantic': robot_description_semantic,
                    'robot_description_kinematics': kinematics_config,
                    'ompl': ompl_config,
                    'planning_pipelines': ['ompl'],
                },
                moveit_controllers,
                trajectory_execution,
                planning_scene_monitor_config,
            ],
        )
        # TF information
        robot_state_publisher = Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description}
            ]
        )
        # Visualization (parameters needed for MoveIt display plugin)
        rviz = Node(
            name='rviz',
            package='rviz2',
            executable='rviz2',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'robot_description_semantic': robot_description_semantic,
                    'robot_description_kinematics': kinematics_config,
                }
            ],
        )
        # Controller manager for realtime interactions
        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters= [
                {'robot_description': robot_description},
                ros_controllers_file
            ],
            output="screen",
        )
        # Startup up ROS2 controllers (will exit immediately)
        controller_names = ['manipulator_joint_trajectory_controller', 'joint_state_controller']
        spawn_controllers = [
            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=[controller],
                output="screen")
            for controller in controller_names
        ]

        return LaunchDescription([
            move_group_node,
            robot_state_publisher,
            ros2_control_node,
            rviz,
            ] + spawn_controllers
        )
    ```

    The bulk of the complexity here is finding and loading all the required parameters using the same helper functions we've used in previous launch files. This launch file defines four nodes to start with the most important one being the `move_group` node. Note that RViz is also started as node, which allows us to initialize it with the same parameters as the move_group node.

 1. Open the new package's `CMakeLists.txt` file and add an installation rule for the `config/` and `launch/` directories underneath the calls to `find_package`:

    ```
    install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
    ```

 1. Rebuild the workspace (`colcon build`) and test a launch file to see if the new package loads without errors:

    ```
    source ~/ros2_ws/install/setup.bash
    ros2 launch myworkcell_moveit_config myworkcell_planning_execution.launch.py
    ```

> _Don't worry too much about how to use RViz.  We'll work through that in the next exercise._
