# Introduction to STOMP

## Motivation
 - Learn how to plan with STOMP through !MoveIt!.


## Information and Resources

- STOMP for [MoveIt!]( http://rosindustrial.org/news/2015/9/25/stomp-for-indigo-presentation-from-the-moveit-community-meeting-3-sept-2015)

- Plugins for [MoveIt!]( http://moveit.ros.org/documentation/plugins/)

## Objectives 
 * Integrate STOMP into !MoveIt! by changing and adding files to a **moveit_config** package.
 * We'll then generate STOMP plans from the [Rviz Motion Planning Plugin](http://docs.ros.org/hydro/api/moveit_ros_visualization/html/doc/tutorial.html)

## Setup
  * Create a workspace
    ```
    mkdir --parent ~/catkin_ws/src
    cd ~/catkin_ws
    catkin init
    catkin build
    source devel/setup.bash
    ```
  * Copy over existing exercise
    ```
    cd ~/catkin_ws/src
    cp -r ~/industrial_training/exercises/4.1 .
    ```
  * Clone industrial_moveit repository into your workspace
    ```
    cd ~/catkin_ws/src
    git clone https://github.com/ros-industrial/industrial_moveit.git
    git checkout melodic-devel
    ```
  * Install Missing Dependencies
    ```
    cd ~/catkin_ws/src/4.1
    rosinstall . .rosinstall
    catkin build
    ```
  * Create a **moveit_config** package created with the [ MoveIt! Setup Assistant ](http://docs.ros.org/hydro/api/moveit_setup_assistant/html/doc/tutorial.html)

## Add STOMP

1. Create a "*stomp_planning_pipeline.launch.xml*" file in the **launch** directory of your **moveit_config** package.  The file should contain the following:
   ``` xml
   <launch>

     <!-- Stomp Plugin for MoveIt! -->
     <arg name="planning_plugin" value="stomp_moveit/StompPlannerManager" />

     <!-- The request adapters (plugins) ORDER MATTERS -->
     <arg name="planning_adapters" value="default_planner_request_adapters/FixWorkspaceBounds
                                          default_planner_request_adapters/FixStartStateBounds
                                          default_planner_request_adapters/FixStartStateCollision
                                          default_planner_request_adapters/FixStartStatePathConstraints" />

     <arg name="start_state_max_bounds_error" value="0.1" />

     <param name="planning_plugin" value="$(arg planning_plugin)" />
     <param name="request_adapters" value="$(arg planning_adapters)" />
     <param name="start_state_max_bounds_error" value="$(arg start_state_max_bounds_error)" />
     <rosparam command="load" file="$(find myworkcell_moveit_config)/config/stomp_planning.yaml"/>

   </launch>
   ```
     
   **!!!** Take notice of the **stomp_planning.yaml** configuration file, this file must exists in moveit_config package.


1. Create the "*stomp_planning.yaml*" configuration file

   This file contains the parameters required by STOMP.  The parameters are specific to each ''planning group'' defined in   the SRDF file.  So if there are three planning groups "manipulator", 
  "manipulator_tool", and "manipulator_rail" then the configuration file defines a specific set of parameters for each  planning group:

   ``` yaml
   stomp/manipulator_rail:
     group_name: manipulator_rail
     optimization:
       num_timesteps: 60
       num_iterations: 40
       num_iterations_after_valid: 0    
       num_rollouts: 30
       max_rollouts: 30 
       initialization_method: 1 #[1 : LINEAR_INTERPOLATION, 2 : CUBIC_POLYNOMIAL, 3 : MININUM_CONTROL_COST
       control_cost_weight: 0.0
     task:
       noise_generator:
         - class: stomp_moveit/NormalDistributionSampling
           stddev: [0.05, 0.8, 1.0, 0.8, 0.4, 0.4, 0.4]
       cost_functions:
         - class: stomp_moveit/CollisionCheck
           collision_penalty: 1.0
           cost_weight: 1.0
           kernel_window_percentage: 0.2
           longest_valid_joint_move: 0.05 
       noisy_filters:
         - class: stomp_moveit/JointLimits
           lock_start: True
           lock_goal: True
         - class: stomp_moveit/MultiTrajectoryVisualization
           line_width: 0.02
           rgb: [255, 255, 0]
           marker_array_topic: stomp_trajectories
           marker_namespace: noisy
       update_filters:
         - class: stomp_moveit/PolynomialSmoother
           poly_order: 6
         - class: stomp_moveit/TrajectoryVisualization
           line_width: 0.05
           rgb: [0, 191, 255]
           error_rgb: [255, 0, 0]
           publish_intermediate: True
           marker_topic: stomp_trajectory
           marker_namespace: optimized
   stomp/manipulator:
     group_name: manipulator
     optimization:
       num_timesteps: 40
       num_iterations: 40
       num_iterations_after_valid: 0    
       num_rollouts: 10
       max_rollouts: 10 
       initialization_method: 1 #[1 : LINEAR_INTERPOLATION, 2 : CUBIC_POLYNOMIAL, 3 : MININUM_CONTROL_COST
       control_cost_weight: 0.0
     task:
       noise_generator:
         - class: stomp_moveit/NormalDistributionSampling
           stddev: [0.05, 0.4, 1.2, 0.4, 0.4, 0.1]
       cost_functions:
         - class: stomp_moveit/CollisionCheck 
           kernel_window_percentage: 0.2
           collision_penalty: 1.0
           cost_weight: 1.0
           longest_valid_joint_move: 0.05
       noisy_filters:
         - class: stomp_moveit/JointLimits
           lock_start: True
           lock_goal: True
         - class: stomp_moveit/MultiTrajectoryVisualization
           line_width: 0.04
           rgb: [255, 255, 0]
           marker_array_topic: stomp_trajectories
           marker_namespace: noisy
       update_filters:
         - class: stomp_moveit/PolynomialSmoother
           poly_order: 5
         - class: stomp_moveit/TrajectoryVisualization
           line_width: 0.02
           rgb: [0, 191, 255]
           error_rgb: [255, 0, 0]
           publish_intermediate: True
           marker_topic: stomp_trajectory
           marker_namespace: optimized      
    ``` 
    **!!!** *Save this file in the* **config** *directory of the moveit_config package*

1. Modify the **move_group.launch** file:
   Open the **move_group.launch** in the launch directory and change the ```pipeline``` parameter value to ```stomp``` as shown below:
   ``` xml
       .
       .
       .
   <!-- move_group settings -->
   <arg name="allow_trajectory_execution" default="true"/>
   <arg name="fake_execution" default="false"/>
   <arg name="max_safe_path_cost" default="1"/>
   <arg name="jiggle_fraction" default="0.05" />
   <arg name="publish_monitored_planning_scene" default="true"/>

   <!-- Planning Functionality -->
   <include ns="move_group" file="$(find myworkcell_moveit_config)/launch/planning_pipeline.launch.xml">
     <arg name="pipeline" value="stomp" />
   </include>

       .
       .
       .
   ```

### Run MoveIt! with STOMP 

1. In a sourced terminal, run the **demo.launch** file:
   ```
   roslaunch myworkcell_moveit_config demo.launch
   ```

1. In Rviz, select robot start and goal positions and plan:
  * In the "Motion Planning" panel, go to the "Planning" tab.
  * Click the "Select Start State" drop-down, select "allZeros" and click "Update"
  * Click the "Select Goal State" drop-down, select "home" and click "Update"
  * Click the "Plan" button and watch the arm move past obstacles to reach the goal position. The blue line shows the tool path.

### Explore STOMP 

1. In Rviz, select other "Start" and "Goal" positions and then hit plan and see the robot move.
2. Display the *Noisy Trajectories* by clicking on the "Marker Array" checkbox in the "Displays" Rviz panel.  Hit the "Plan" button again and you'll see the noisy trajectory markers as yellow lines.

> Note:
>  STOMP explores the workspace by generating a number of noisy trajectories as a result of applying noise onto the current trajectory.  The degree of noise applied can be be changed by adjusting the "stddev" parameters in the "*stomp_config.yaml*" file.  Larger "stddev" values correspond to larger motions of the joints.
>

 

### Configure STOMP 
 
 We'll now change the parameters in the **stomp_config.yaml** and see what effect those changes have on the planning.
 
1.  Ctrl-C in the terminal where you ran the **demo.launch** file earlier to stop the **move_group** planning node.
1.  Locate and open up the **stomp_config.yaml** with your preferred editor.
1.  Under the "manipulator_rail" group, take notice of the values assigned to "stddev" parameter.  Each value is the amplitude of the noise applied to the joint at that position in the array.  For instance, the leftmost value in the array will be the value used to set the noise of the first joint "rail_to_base"; which moves the rail along the x direction.  Since the "rail_to_base" is a prismatic joint then its units are in meters; for revolute joints the units are radians.
1. Change the "stddev" values (preferably one entry at a time), save the file and rerun the "demo.launch" file in the terminal.
1. Go back to the Rviz window and select arbitrary "Start" and "Goal" positions to see what effect your changes have had on the planning performance.

### More info on the STOMP parameters

 The [STOMP wiki]() explains these parameter in more detail.
