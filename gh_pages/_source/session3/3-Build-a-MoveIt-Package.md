# Build a MoveIt! Package
>In this exercise, we will create a MoveIt! package for an industrial robot. This package creates the configuration and launch files required to use a robot with the MoveIt! Motion-Control nodes. In general, the MoveIt! package does not contain any C++ code.

> _NOTE: The "MoveIt Setup Assistant" is not available in ROS 2 distributions before Humble. A workaround for earlier distributions is to use the ROS 1 version, have it output a ROS1-compatible package, and to adapt the output for ROS2 (as we did in [previous versions](https://industrial-training-master.readthedocs.io/en/foxy/_source/session3/ros2/3-Build-a-MoveIt-Package.html) of this training)._

## Motivation
MoveIt! is a free-space motion planning framework for ROS. It’s an incredibly useful and easy-to-use tool for planning motions between two points in space without colliding with anything. Under the hood MoveIt is quite complicated, but (unlike most ROS libraries) it has a really nice GUI Wizard to get you started.

## Reference Example
[Using MoveIt with ROS-I](http://wiki.ros.org/Industrial/Tutorials/Create_a_MoveIt_Pkg_for_an_Industrial_Robot)

## Further Information and Resources
[MoveIt Setup Assistant Tutorial](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

## Scan-N-Plan Application: Problem Statement
In this exercise, you will generate a MoveIt package for the UR5 workcell you built in a previous step. This process will mostly involve running the MoveIt! Setup Assistant. At the end of the exercise you should have the following:

 1. A new package called `myworkcell_moveit_config`

 1. A moveit configuration with one group ("manipulator"), that consists of the kinematic chain between the UR5’s `base_link` and `tool0`.

## Scan-N-Plan Application: Guidance

### Create a Base Package using the Setup Assistant

 1. Install the MoveIt 2 Humble packages:

    ```
    sudo apt install ros-humble-moveit
    ```

 1. Source your workspace and start the MoveIt! Setup Assistant (don't forget auto-complete with tab):

    ```
    ros2 launch moveit_setup_assistant setup_assistant.launch.py
    ```

 1. Select "Create New MoveIt Configuration Package", select the `workcell.urdf.xacro` in your workspace's _myworkcell_support_ package, then "Load File".

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

       1. Set the kinematics solver to `KDLKinematicsPlugin`
       1. Set the default OMPL planner to `RRTConnect`

    1. Create a few named positions (e.g. "home", "allZeros", etc.) to test with motion-planning.

    1. Don't worry about adding end effectors, passive joints, or ros2_control URDF modifications for this exercise.
    
    1. In the "ROS 2 Controllers" tab, use the "Auto Add JointTrajectoryController" button to define a basic _JointTrajectoryController_ controller for the entire UR5 arm.

    1. In the "MoveIt Controllers" tab,use the "Auto Add FollowJointsTrajectory" button to define a basic _FollowJointTrajectory_ controller for the UR5 arm.
    
    1. Skip the "Perception" and "Launch Files" tabs.

    1. Enter author / maintainer info.

       _Yes, it's required, but doesn't have to be valid._

    1. Generate a new package and name it `myworkcell_moveit_config`.
       * make sure to create the package inside your `ros2_ws/src` directory

## Update the Setup Assistant Output

 The outcome of the Setup Assistant is a new ROS2 package that contains a large number of launch and configuration files.  We need to add just a bit more to customize this general-purpose package for our application.

 1. Create a new file in your `myworkcell_moveit_config` package named `config/ompl_planning.yaml`. Paste in the following code:

    ```
    manipulator:
      default_planner_config: RRTConnect
      planner_configs:
        - AnytimePathShortening
        - SBL
        - EST
        - LBKPIECE
        - BKPIECE
        - KPIECE
        - RRT
        - RRTConnect
        - RRTstar
        - TRRT
        - PRM
        - PRMstar
        - FMT
        - BFMT
        - PDST
        - STRIDE
        - BiTRRT
        - LBTRRT
        - BiEST
        - ProjEST
        - LazyPRM
        - LazyPRMstar
        - SPARS
        - SPARStwo
      projection_evaluator: joints(shoulder_pan_joint,shoulder_lift_joint)
      longest_valid_segment_fraction: 0.005

    planning_plugin: 'ompl_interface/OMPLPlanner'
    request_adapters: >-
        default_planner_request_adapters/AddTimeOptimalParameterization
        default_planner_request_adapters/FixWorkspaceBounds
        default_planner_request_adapters/FixStartStateBounds
        default_planner_request_adapters/FixStartStateCollision
        default_planner_request_adapters/FixStartStatePathConstraints
    start_state_max_bounds_error: 0.1

    ```
    
    Make sure your editor does not change the amount of whitespace at the front of these lines.

 1. You may wish to examine some key files in your new package:

 * `workcell.srdf`: This file defines additional information about the workspace links and joints such as planning groups, end-effectors, virtual joints, and allowed collisions.

 * `kinematics.yaml`: These parameters define the kinematics solver used to find joint angles for a given cartesian position.

 * `ompl_planning.yaml`: This informs MoveIt on what planner profiles are available for our manipulator and provides some additional configuration parameters.

 * `moveit_controllers.yaml`: This file will configure MoveIt to use a controller for joint trajectory execution provided by `ros2_control`.

 * `ros2_controllers.yaml`: These parameters configure the ROS 2 Control nodes at startup.

 1. Rebuild the workspace (`colcon build`) and test a launch file to see if the new package loads without errors:

    ```
    source ~/ros2_ws/install/setup.bash
    ros2 launch myworkcell_moveit_config demo.launch.py
    ```

> _Don't worry too much about how to use RViz.  We'll work through that in the next exercise._

## Challenge Exercises
* Imagine you have misrepresented your robot and need to add an additional joint. Would it be faster to make minor edits to all configuration files or to re-do the MoveIt package creation process? Try each method and compare.