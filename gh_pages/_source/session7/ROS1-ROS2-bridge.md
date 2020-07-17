# ROS1-ROS2 Bridge Demo

## Introduction

This is a system integration exercise to demonstrate operation of the ROS1-ROS2 topic and service
bridge. Using the bridge does not require anything different when developing either ROS1 or ROS2
software and so we will not worry about writing code for this exercise.

This demo is an example of a system where a ROS2 application needs to call a planner that only
exists as a ROS1 package. Specifically, this demo is calling the Descartes motion planner,
as seen in exercise 4.1. On the ROS1 side, services are provided to generate motion plans and to
execute them. In order to use these custom services, the bridge needs to be recompiled with the
definitions in the build environment, which is the bulk of the complexity in this exercise.

## Building the Demo

This exercise uses the ROS1 bridge to call ROS nodes from ROS2 nodes and therefore the build
procedure is somewhat involved.

### Create a ROS workspace for exercise 4.1

1.  Create a catkin workspace for the exercise 4.1 ROS packages and dependencies
    ```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    ```

1.  Create a symlink to exercise 4.1 in the training repo
    ```
    cd ~/catkin_ws/src
    ln -s ~/industrial_training/exercises/4.1/src demo
    ```

1.  Clone additional dependencies
    ```
    git clone https://github.com/ros-industrial-consortium/descartes.git
    git clone https://github.com/ros-industrial/universal_robot.git
    git clone https://github.com/ros-industrial/fake_ar_publisher.git
    ```

1.  Source and build exercise
    ```
    . /opt/ros/melodic/setup.bash
    catkin build
    ```

### Create the ROS2 workspace

1.  Create a colcon workspace for the exercise 7.2 ROS packages and dependencies
    ```
    mkdir -p ~/colcon_ws/src
    cd ~/colcon_ws/src
    ```

1.  Create a symlink to exercise 7.2
    ```
    ln -s ~/industrial_training/exercises/7.2/src demo
    ```

1.  Source and build
    ```
    cd ~/colcon_ws/src
    . /opt/ros/eloquent/setup.bash
    colcon build
    ```

### Build the ROS1 Bridge 

1.  Create the ros1_bridge workspace. We build the bridge in a separate workspace because it needs
    to see both ROS1 and ROS2 packages in its environment and we want to make sure our application
    workspaces only see the packages from the distribution they are in.
    ```
    mkdir -p ~/ros1_bridge_ws/src
    cd ~/ros1_bridge_ws/src
    ```

1.  Clone the ROS1 bridge into your ROS2 workspace, selecting the branch that matches your ROS
    release
    ```
    git clone -b eloquent https://github.com/ros2/ros1_bridge.git
    ```

1.  Source ROS1 and ROS2 setup bash files. This is one of the only times you'll want to mix setup
    files from different ROS distributions.
    ```
    . ~/catkin_ws/devel/setup.bash
    . ~/colcon_ws/install/setup.bash
    ```

    You should see a warning about the `ROS_DISTRO` variable being previously set to a different
    value.  Now the environment contains references to both distributions which can be verified by
    observing the CMake path:
    ```
    echo $CMAKE_PREFIX_PATH | tr ':' '\n'
    ```

1.  Build the bridge. This may take a while since it is creating mappings between all known message
    and service types.
    ```
    colcon build --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=FALSE
    ```

1.  List the mapped message and service pairs
    ```
    source install/local_setup.bash
    ros2 run ros1_bridge dynamic_bridge --print-pairs
    ```
    This should list the custom `myworkcell_msgs(ROS2) <-> myworkcell_core` mapped services

---
## Run the Demo

### Run the ROS nodes

1.  In terminal sourced to your ROS catkin_ws workspace start the roscore
    ```
    roscore
    ```

2.  In another sourced terminal, run the following launch file:
    ```
    roslaunch myworkcell_support ros2_setup.launch
    ```

    This will launch the motion planning and motion execution nodes is ROS.  Rviz will come up as well.

### Run the ROS1 bridge

1.  In your ros1_bridge_ws ROS2 workspace, source the workspace if you haven't
    ```
    cd ~/ros1_bridge_ws
    source install/setup.bash
    ```

1.  Export the _ROS_MASTER_URI_ environment variable
    ```
    export ROS_MASTER_URI=http://localhost:11311
    ```

1.  Run the bridge
    ```
    ros2 run ros1_bridge dynamic_bridge
    ```

### Run the ROS2 nodes

1.  Open a new terminal and source your ROS2 workspace
    ```
    cd ~/colcon_ws
    source install/setup.bash
    ```

1.  Run the python launch file that starts the ROS2 nodes for this application
    ```
    ros2 launch myworkcell_core workcell.launch.py
    ```
    If the program succeeds you should see the following output:

    > [myworkcell_node-2] Got base_frame parameter world  
    > [myworkcell_node-2] Waiting for client /localize_part  
    > [myworkcell_node-2] Waiting for client /plan_path  
    > [myworkcell_node-2] Waiting for client /move_to_pose  
    > [myworkcell_node-2] Waiting for client /execute_trajectory  
    > [myworkcell_node-2] Found all services  
    > [myworkcell_node-2] Requesting pose in base frame: world  
    > [myworkcell_node-2] Part localized  
    > [myworkcell_node-2] Planning trajectory  
    > [myworkcell_node-2] Executing trajectory  
    > [myworkcell_node-2] Trajectory execution complete  

    You should also see the robot moving accordingly in the Rviz window.

---
## Issues:
  - **Problem**: The application only runs successfully once, the next run will fail due to a tf
    lookup operation in the vision_node.  This likely happens due to the bridge removing the
    bridging for all `/tf` topics after the application ends.  Solution:

    **Solution**: Restart the `ros1_bridge` and then the `workcell.launch.py` application.
