# Build a MoveIt! Package
>In this exercise, we will create a MoveIt! package for an industrial robot. This package creates the configuration and launch files required to use a robot with the MoveIt! Motion-Control nodes. In general, the MoveIt! package does not contain any C++ code.


## Motivation
MoveIt! is a free-space motion planning framework for ROS. It’s an incredibly useful and easy-to-use tool for planning motions between two points in space without colliding with anything. Under the hood MoveIt is quite complicated, but (unlike most ROS libraries) it has a nice GUI Wizard to get you started.

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

1. Open a NEW terminal and set up your ROS1 workspace to run the MoveIt Setup Assistant.  Put copies of the required URDF packages (`ur_description`, `myworkcell_support`) inside this ROS1 workspace, to make them visible to the Setup Assistant:

   ```
   cd ~/ros2_ws/src
   git clone https://github.com/ros-industrial/universal_robot.git
   cp -r ~/industrial_training/exercises/3.3/ros1/src/myworkcell_support ~/ros2_ws/src
   
   <edit ~/ros2_ws/src/myworkcell_support/package.xml & CMakeLists.txt and remove all references to myworkcell_core>
   
   cd ~/ros2_ws/
   source /opt/ros/noetic/setup.bash
   catkin build
   source ~/ros2_ws/devel/setup.bash
   ```

 1. Start the MoveIt! Setup Assistant (don't forget auto-complete with tab):

    ```
    ros2 launch moveit_setup_assistant setup_assistant.launch.py
    ```

 1. Select "Create New MoveIt Configuration Package", and select the `workcell.urdf.xacro` in the workspace's _myworkcell_support_ package, then "Load File".

 1. Work your way through the tabs on the left from the top down.

    1. In "Self-Collisions", move the slider "Sampling Density" to the highest possible value and generate a self-collision matrix. After a few seconds the MoveIt! Setup Assistant will present you the results of the computation. 

    1. Add a fixed virtual base joint. e.g.

       ```
       name = 'FixedBase' (arbitrary)
       child = 'world' (should match the URDF root link)
       parent = 'world' (reference frame used for motion planning)
       type = 'fixed'
       ```

    1. Add a planning group called `manipulator` that names the kinematic chain between `base_link` and `tool0`. Note: Follow [ROS naming guidelines/requirements](http://wiki.ros.org/ROS/Patterns/Conventions) and don't use any whitespace, anywhere.

       a. Set the kinematics solver to `KDLKinematicsPlugin`

       b. Set the default OMPL planner to `RRTConnect`

       c. Using "Add Kin. Chain" the Base Link and Tip Link can be specified as `base_link` and `tool0` 

       d. after saving the planning group, select `Joints` and `Èdit Selected`, add `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint` and `wrist_[1-3]_joint` and save again. 


    1. In `Robot Poses`, create a few named positions (e.g. "home", "allZeros", etc.) to test with motion-planning.

    1. Don't worry about adding end effectors/grippers or passive joints for this exercise.

    1. Skip the ros2_control URDF Model pane, as we use the ros2_control model provided by UR Robot Driver. 
    
    1. In the "ROS 2 Controllers" tab, use the "Auto Add JointTrajectoryController Controllers For Each Planning Group" button to define a basic _joint_trajectory_controller_ for the entire UR5 arm. Rename the `manipulator_controller` to `joint_trajectory_controller`

    1. In the "Moveit Controllers" tab, use the "Auto Add FollowJointsTrajectory Controllers For Each Planning Group" button to define a basic _follow_joint_controller_ for the entire UR5 arm. Rename the `manipulator_controller` to `joint_trajectory_controller`

    
    1. Skip the Simulation and Launch Files tabs.

    1. Enter author / maintainer info.

       _Yes, it's required, but doesn't have to be valid_

    1. Generate a new package and name it `myworkcell_moveit_config`.
       * make sure to create the package inside your `ros1_ws/src` directory

## Create ROS2 Package from Setup Assistant Output

 The outcome of the Setup Assistant is a new ROS1 package that contains a large number of launch and configuration files.  We need to add a few files to customize this general-purpose package for our application.

 > _Note: The world of MoveIt is continually evolving in ROS2; this is currently just one way to get a ROS2 package going and is not presented as the "correct" way._



* You need to set up cthe orrect initial positions for the robot. To do so overwrite the content of the file `initial_positions.yaml` with the following lines: 
```
initial_positions:
  elbow_joint: -1.57
  shoulder_lift_joint: -1.57
  shoulder_pan_joint: -1.57
  wrist_1_joint: -1.57
  wrist_2_joint: 1.57
  wrist_3_joint: 0.0

elbow_joint: -1.57
shoulder_lift_joint: -1.57
shoulder_pan_joint: -1.57
wrist_1_joint: -1.57
wrist_2_joint: 1.57
wrist_3_joint: 0.0
```


* Next you need to setup the correct joint limits. To do so overwrite your `joint_limits.yaml` with: 

```
# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or augmented as needed

# For beginners, we downscale velocity and acceleration limits.
# You can always specify higher scaling factors (<= 1.0) in your motion requests.  # Increase the values below to 1.0 to always move at maximum speed.
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1

# Specific joint properties can be changed with the keys [max_position, min_position, max_velocity, max_acceleration]
# Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]
joint_limits:
  elbow_joint:
    has_velocity_limits: true
    max_velocity: 3.1415926535897931
    has_acceleration_limits: false
    max_acceleration: 0
  shoulder_lift_joint:
    has_velocity_limits: true
    max_velocity: 3.1415926535897931
    has_acceleration_limits: false
    max_acceleration: 0
  shoulder_pan_joint:
    has_velocity_limits: true
    max_velocity: 3.1415926535897931
    has_acceleration_limits: false
    max_acceleration: 0
  wrist_1_joint:
    has_velocity_limits: true
    max_velocity: 3.1415926535897931
    has_acceleration_limits: false
    max_acceleration: 0
  wrist_2_joint:
    has_velocity_limits: true
    max_velocity: 3.1415926535897931
    has_acceleration_limits: false
    max_acceleration: 0
  wrist_3_joint:
    has_velocity_limits: true
    max_velocity: 3.1415926535897931
    has_acceleration_limits: false
    max_acceleration: 0

```

* Then we need to fix the ROS2 controllers configuration. The UR robot has many more controllers than the ones Moveit knows about. Therefore copy the following contents into your `ros2_controllers.yaml`.

```
controller_manager:
  ros__parameters:
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    io_and_status_controller:
      type: ur_controllers/GPIOController

    speed_scaling_state_broadcaster:
      type: ur_controllers/SpeedScalingStateBroadcaster

    force_torque_sensor_broadcaster:
      type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    scaled_joint_trajectory_controller:
      type: ur_controllers/ScaledJointTrajectoryController

    forward_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

    forward_position_controller:
      type: position_controllers/JointGroupPositionController
    
    update_rate: 500  # Hz


speed_scaling_state_broadcaster:
  ros__parameters:
    state_publish_rate: 100.0


force_torque_sensor_broadcaster:
  ros__parameters:
    sensor_name: tcp_fts_sensor
    state_interface_names:
      - force.x
      - force.y
      - force.z
      - torque.x
      - torque.y
      - torque.z
    frame_id: tool0
    topic_name: ft_data


joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 100.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.2
      goal_time: 0.0
      shoulder_pan_joint: { trajectory: 0.2, goal: 0.1 }
      shoulder_lift_joint: { trajectory: 0.2, goal: 0.1 }
      elbow_joint: { trajectory: 0.2, goal: 0.1 }
      wrist_1_joint: { trajectory: 0.2, goal: 0.1 }
      wrist_2_joint: { trajectory: 0.2, goal: 0.1 }
      wrist_3_joint: { trajectory: 0.2, goal: 0.1 }


scaled_joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    state_publish_rate: 100.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.2
      goal_time: 0.0
      shoulder_pan_joint: { trajectory: 0.2, goal: 0.1 }
      shoulder_lift_joint: { trajectory: 0.2, goal: 0.1 }
      elbow_joint: { trajectory: 0.2, goal: 0.1 }
      wrist_1_joint: { trajectory: 0.2, goal: 0.1 }
      wrist_2_joint: { trajectory: 0.2, goal: 0.1 }
      wrist_3_joint: { trajectory: 0.2, goal: 0.1 }

forward_velocity_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    interface_name: velocity

forward_position_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint

```

* Finally we need to add the configuration for the ompl planner. To do so add ``ompl_planning.yaml`` to your config folder with the following lines: 

```
planning_plugin: ompl_interface/OMPLPlanner
request_adapters: >-
  default_planner_request_adapters/AddTimeOptimalParameterization
  default_planner_request_adapters/ResolveConstraintFrames
  default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStateCollision
  default_planner_request_adapters/FixStartStatePathConstraints
start_state_max_bounds_error: 0.1
planner_configs:
  SBLkConfigDefault:
    type: geometric::SBL
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
  ESTkConfigDefault:
    type: geometric::EST
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0 setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability. default: 0.05
  LBKPIECEkConfigDefault:
    type: geometric::LBKPIECE
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    border_fraction: 0.9  # Fraction of time focused on boarder default: 0.9
    min_valid_path_fraction: 0.5  # Accept partially valid moves above fraction. default: 0.5
  BKPIECEkConfigDefault:
    type: geometric::BKPIECE
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    border_fraction: 0.9  # Fraction of time focused on boarder default: 0.9
    failed_expansion_score_factor: 0.5  # When extending motion fails, scale score by factor. default: 0.5
    min_valid_path_fraction: 0.5  # Accept partially valid moves above fraction. default: 0.5
  KPIECEkConfigDefault:
    type: geometric::KPIECE
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability. default: 0.05
    border_fraction: 0.9  # Fraction of time focused on boarder default: 0.9 (0.0,1.]
    failed_expansion_score_factor: 0.5  # When extending motion fails, scale score by factor. default: 0.5
    min_valid_path_fraction: 0.5  # Accept partially valid moves above fraction. default: 0.5
  RRTkConfigDefault:
    type: geometric::RRT
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability? default: 0.05
  RRTConnectkConfigDefault:
    type: geometric::RRTConnect
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
  RRTstarkConfigDefault:
    type: geometric::RRTstar
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability? default: 0.05
    delay_collision_checking: 1  # Stop collision checking as soon as C-free parent found. default 1
  TRRTkConfigDefault:
    type: geometric::TRRT
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()
    goal_bias: 0.05  # When close to goal select goal, with this probability? default: 0.05
    max_states_failed: 10  # when to start increasing temp. default: 10
    temp_change_factor: 2.0  # how much to increase or decrease temp. default: 2.0
    min_temperature: 10e-10  # lower limit of temp change. default: 10e-10
    init_temperature: 10e-6  # initial temperature. default: 10e-6
    frountier_threshold: 0.0  # dist new state to nearest neighbor to disqualify as frontier. default: 0.0 set in setup()
    frountierNodeRatio: 0.1  # 1/10, or 1 nonfrontier for every 10 frontier. default: 0.1
    k_constant: 0.0  # value used to normalize expression. default: 0.0 set in setup()
  PRMkConfigDefault:
    type: geometric::PRM
    max_nearest_neighbors: 10  # use k nearest neighbors. default: 10
  PRMstarkConfigDefault:
    type: geometric::PRMstar
arm:
  planner_configs:
    - SBLkConfigDefault
    - ESTkConfigDefault
    - LBKPIECEkConfigDefault
    - BKPIECEkConfigDefault
    - KPIECEkConfigDefault
    - RRTkConfigDefault
    - RRTConnectkConfigDefault
    - RRTstarkConfigDefault
    - TRRTkConfigDefault
    - PRMkConfigDefault
    - PRMstarkConfigDefault
  ##Note: commenting the following line lets moveit chose RRTConnect as default planner rather than LBKPIECE
  projection_evaluator: joints(shoulder_pan_joint,shoulder_lift_joint)
  longest_valid_segment_fraction: 0.01

```
* The UR robot comes with a quite complex xacro file that needs many arguments which **Moveit Setup Assistant** does not know about. Replace the contents of myworkcell.urdf.xacro with the following content.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="urworkcell">

    <xacro:arg name="name" default="myworkcell" />
    <xacro:arg name="ur_type" default="ur5" />
    <xacro:arg name="prefix" default="" />
    <xacro:arg name="script_filename" default="$(find ur_robot_driver)/resources/ros_control.urscript" />
    <xacro:arg name="output_recipe_filename" default="$(find ur_robot_driver)/resources/rtde_output_recipe.txt" />
    <xacro:arg name="input_recipe_filename" default="$(find ur_robot_driver)/resources/rtde_input_recipe.txt" />
    <xacro:arg name="use_fake_hardware" default="true" />
    <xacro:arg name="fake_sensor_commands" default="true" />
    <xacro:arg name="robot_ip" default="192.168.56.2" />
    <xacro:arg name="prefix" default="" />

    <xacro:include filename="$(find myworkcell_support)/urdf/myworkcell.urdf.xacro" />
</robot>
```

    These parameters configure the controller nodes at startup.

    Rename the file `demo.launch.py`in the launch folder to `myworkcell_planning_execution.launch.py`

 1. Rebuild the workspace (`colcon build`) and now you are ready to make use of the newly created MoveIt! Package. 

  You can try out your newly created MoveIt! package by launching the following:   
  `ros2 launch myworkcell_moveit_config myworkcell_planning_execution.launch`   
  RViz should start and the UR5 should be visualized.

> _Don't worry too much about how to use RViz.  We'll work through that in the next exercise._

## Challenge Exercises
* Imagine you have misrepresented your robot and need to add an additional joint. Would it be faster to make minor edits to all configuration files or to re-do the MoveIt package creation process? Try each method and compare.