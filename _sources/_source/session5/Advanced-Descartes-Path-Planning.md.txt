# Advanced Descartes Path Planning
>In this exercise, we will use advanced features with Descartes to solve a complex path for part being held by the robot which gets processed by a stationary tool.

## Motivation
MoveIt! is a framework meant primarily for performing "free-space" motion where the objective is to move a robot from point A to point B and you don't particularly care about how that gets done. These types of problems are only a subset of frequently performed tasks. Imagine any manufacturing ''process'' like welding or painting. You very much care about where that tool is pointing the entire time the robot is at work.

This tutorial introduces you to Descartes, a ''Cartesian'' motion planner meant for moving a robot along some process path. It's only one of a number of ways to solve this kind of problem, but it's got some neat properties:
 * It's deterministic and globally optimum (to a certain search resolution).
 * It can search redundant degrees of freedom in your problem (say you have 7 robot joints or you have a process where the tool's Z-axis rotation doesn't matter).

## Reference Example
[Descartes Tutorial](http://wiki.ros.org/descartes/Tutorials/Getting%20Started%20with%20Descartes)

## Further Information and Resources
[Descartes Wiki](http://wiki.ros.org/descartes)

APIs:
 * [descartes_core::PathPlannerBase](http://docs.ros.org/indigo/api/descartes_core/html/classdescartes__core_1_1PathPlannerBase.html)
 * [descartes_planner::DensePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html)
 * [descartes_planner::SparsePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1SparsePlanner.html)

## Scan-N-Plan Application: Problem Statement
In this exercise, you will add two new nodes, two xacro, and config file to your Scan-N-Plan application, that:
 1. Takes the config file `puzzle_bent.csv` and creates a descartes trajectory where the part is held by the robot and manipulated around a stationary tool.
 1. Produces a joint trajectory that commands the robot to trace the perimeter of the marker (as if it is dispensing adhesive).

## Scan-N-Plan Application: Guidance
In the interest of time, we've included several files:
 1. The first is a template node `adv_descartes_node.cpp` where most of the exercise is spent creating the complicated trajectory for deburring a complicated part.
 1. The second node `adv_myworkcell_node.cpp` which is a duplicate of the `myworkcell_node.cpp` where it calls a service within the `adv_descartes_node.cpp`.
 1. The config file `puzzle_bent.csv` which contains the path relative to the part coordinate system.
 1. The two xacro files `puzzle_mount.xacro` and `grinder.xacro` which are used to update the urdf/xacro `workcell.xacro` file.

Left to you are the details of:
 1. Updating the workcell.xacro file to include the two new xacro files.
 1. Define a new move group in your moveit_config package called "puzzle".
 1. Defining a series of Cartesian poses that comprise a robot “path”.
 1. Translating those paths into something Descartes can understand.

### Setup workspace
 1. If have not went through sessions 1-4, copy the src directory from exercise 4.1. Otherwise move to the next step.

    ```bash
    mkdir ~/catkin_ws
    cd ~/catkin_ws
    cp -r ~/industrial_training/exercises/4.1/src .
    cd src
    git clone https://github.com/jmeyer1292/fake_ar_publisher.git
    git clone -b kinetic-devel https://github.com/ros-industrial-consortium/descartes.git
    sudo apt install ros-kinetic-ur-kinematics
    sudo apt install ros-kinetic-ur-description
    ```

 1. Copy over the `adv_descartes_node_unfinished.cpp` into your core package's src/ folder and rename it `adv_descartes_node.cpp`.

    ```bash
    cp ~/industrial_training/exercises/5.0/src/adv_descartes_node_unfinished.cpp myworkcell_core/src/adv_descartes_node.cpp
    ```

 1. Create rules in the `myworkcell_core` package's `CMakeLists.txt` to build a new node called `adv_descartes_node`.  As in previous exercises, add these lines near similar lines in the template file (not as a block as shown below).
     
    ```cmake
    add_executable(adv_descartes_node src/adv_descartes_node.cpp)
    add_dependencies(adv_descartes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(adv_descartes_node ${catkin_LIBRARIES})
    ```

 1. Copy over the `adv_myworkcell_node.cpp` into your core package's src/ folder.

    ```bash
    cp ~/industrial_training/exercises/5.0/src/myworkcell_core/src/adv_myworkcell_node.cpp myworkcell_core/src/adv_myworkcell_node.cpp
    ```

 1. Create rules in the `myworkcell_core` package's `CMakeLists.txt` to build a new node called `adv_myworkcell_node`.  As in previous exercises, add these lines near similar lines in the template file (not as a block as shown below).
     
    ```cmake
    add_executable(adv_myworkcell_node src/adv_myworkcell_node.cpp)
    add_dependencies(adv_myworkcell_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(adv_myworkcell_node ${catkin_LIBRARIES})
    ```

 1. Copy over the necessesary config file:
    ``` bash
    mkdir ~/catkin_ws/src/myworkcell_core/config
    cp ~/industrial_training/exercises/5.0/src/myworkcell_core/config/puzzle_bent.csv myworkcell_core/config/puzzle_bent.csv
    cp ~/industrial_training/exercises/5.0/src/myworkcell_support/urdf/grinder.xacro myworkcell_support/urdf/grinder.xacro
    cp ~/industrial_training/exercises/5.0/src/myworkcell_support/urdf/puzzle_mount.xacro myworkcell_support/urdf/puzzle_mount.xacro
    mkdir ~/catkin_ws/src/myworkcell_support/meshes
    cp ~/industrial_training/exercises/5.0/src/myworkcell_support/meshes/* myworkcell_support/meshes/ 
    ```

### Update your workcell.xacro file.
 1. Add two include tags for grinder.xacro and puzzle_mount.xacro.
 1. Attach grinder to the **world** link with the origin: 
    ``` xml
    <origin xyz="0.0 -0.4 0.6" rpy="0 3.14159 0"/>
    ```
 1. Attach the puzzle mount the robot link **tool0** with the origin:
    ``` xml
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    ```
 1. Launch the demo.launch file within your moveit_config package to verify the workcell. There should be a grinder sticking out of the table and a puzzle piece shaped part attached to the robot.
    ``` bash
    roslaunch myworkcell_moveit_config demo.launch
    ```

### Add new move group to your moveit_config package.
 1. Run moveit setup assistant and add a new move group with the kinematic chain **base_link -> part**. Note: Since you added geometry you should also regenerate the allowed collision matrix.
    ``` bash
    roslaunch moveit_setup_assistant setup_assistant.launch 
    ```

### Complete Advanced Descartes Node
 1. First, the function `EigenSTL::vector_Affine3d makePuzzleToolPoses()` needs to be completed. The file path for **puzzle_bent.csv** is need. Don't not provide the full path, please use the ros tool `ros::package::getPath()` for getting the path of a package.
 1. Next, the function `std::vector<descartes_core::TrajectoryPtPtr>
  makeDescartesTrajectory(const EigenSTL::vector_Affine3d& path)` needs to be completed. The transform between **world** and **grinder_frame** needs to be found. Also Each point needs to have a tolerance set for the z-axis to +/- PI;

### Update the setup.launch file.
 1. Update the file to take an argument **adv** so that either the basic or advanced descartes node can be launched.

### Test Full Application

 1. Run the new setup file, then your main advanced workcell node:

    ``` bash
    roslaunch myworkcell_support setup.launch adv:=true
    rosrun myworkcell_core adv_myworkcell_node
    ```

    It's difficult to see what's happening with the rviz planning-loop always running.  Disable this loop animation in rviz (Displays -> Planned Path -> Loop Animation), then rerun `adv_myworkcell_node`.

### Solutions
 * 5.2 Update your workcell.xacro file.
   ``` xml
   <include filename="$(find myworkcell_support)/urdf/puzzle_mount.xacro" />
   <include filename="$(find myworkcell_support)/urdf/grinder.xacro" />   
   
   <joint name="world_to_grinder" type="fixed">
    <parent link="world"/>
    <child link="grinder_frame"/>
    <origin xyz="0.0 -0.4 0.6" rpy="0 3.14159 0"/>
   </joint> 

   <joint name="robot_tool" type="fixed">
    <parent link="tool0"/>
    <child link="ee_mount"/>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
   </joint>
   ```
 * 5.4 Complete Advanced Descartes Node

   **Step 1:**
   ``` c++
   std::string filename = ros::package::getPath("myworkcell_core") + "/config/puzzle_bent.csv";
   ```

   **Step 2:**
   ``` c++
   listener_.lookupTransform("world", "grinder_frame", ros::Time(0), grinder_frame);
   tf::transformTFToEigen(grinder_frame, gf);
   ```

   **Step 3:**
   ``` c++
   tool_pt.orientation_tolerance.z_lower -= M_PI;
   tool_pt.orientation_tolerance.z_upper += M_PI;
   ```

 * 5.5 Update the setup.launch file.
   ``` xml
   <launch>
     <arg name="adv" default="0" />
     <include file="$(find myworkcell_moveit_config)/launch/myworkcell_planning_execution.launch"/>
     <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" />
     <node name="vision_node" type="vision_node" pkg="myworkcell_core" output="screen"/>
     <node name="descartes_node" type="descartes_node" pkg="myworkcell_core" output="screen" unless="$(arg adv)"/>
     <node name="adv_descartes_node" type="adv_descartes_node" pkg="myworkcell_core" output="screen" if="$(arg adv)"/>
   </launch>
   ```
