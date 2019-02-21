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
 1. The second node, `adv_myworkcell_node.cpp`, is a duplicate of the `myworkcell_node.cpp` that has been updated to call the `adv_plan_path` service provided by `adv_descartes_node.cpp`.
 1. The config file `puzzle_bent.csv` which contains the path relative to the part coordinate system.
 1. The two xacro files `puzzle_mount.xacro` and `grinder.xacro` which are used to update the urdf/xacro `workcell.xacro` file.

Left to you are the details of:
 1. Updating the workcell.xacro file to include the two new xacro files.
 1. Updating the moveit_config package to define a new Planning Group for this exercise, including the new end-effector links.
 1. Defining a series of Cartesian poses that comprise a robot “path”.
 1. Translating those paths into something Descartes can understand.

### Setup workspace
 1. This exercise uses the same workspace from the Basic Training course.  If you don't have this workspace (completed through Exercise 4.1), copy the completed reference code and pull in other required dependencies as shown below. Otherwise move to the next step.

    ```bash
    mkdir ~/catkin_ws
    cd ~/catkin_ws
    cp -r ~/industrial_training/exercises/4.1/src .
    cd src
    git clone https://github.com/jmeyer1292/fake_ar_publisher.git
    git clone -b melodic-devel https://github.com/ros-industrial-consortium/descartes.git
    git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git
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
    cp ~/industrial_training/exercises/5.0/src/myworkcell_core/src/adv_myworkcell_node.cpp myworkcell_core/src/
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
    cp ~/industrial_training/exercises/5.0/src/myworkcell_core/config/puzzle_bent.csv myworkcell_core/config/
    cp ~/industrial_training/exercises/5.0/src/myworkcell_support/urdf/grinder.xacro myworkcell_support/urdf/
    cp ~/industrial_training/exercises/5.0/src/myworkcell_support/urdf/puzzle_mount.xacro myworkcell_support/urdf/
    mkdir ~/catkin_ws/src/myworkcell_support/meshes
    cp ~/industrial_training/exercises/5.0/src/myworkcell_support/meshes/* myworkcell_support/meshes/ 
    ```

 1. Add new package dependencies:
    * Add `tf_conversions` to `CMakeLists.txt` (2 places) and `package.xml` (1 place)

### Update your workcell.xacro file.
 1. Add two `<include>` tags for the new `grinder.xacro` and `puzzle_mount.xacro` files.
 1. Attach the grinder to the **world** link with the following offset: 
    ``` xml
    <origin xyz="0.0 -0.4 0.6" rpy="0 3.14159 0"/>
    ```
    * Look in the `grinder.xacro` file to locate the appropriate `<child_link>` name.
    * Copy one of the other `<joint>` tag definitions and modify as appropriate.
 1. Attach the puzzle mount to the robot's **tool0** frame with the following offset:
    ``` xml
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    ```
    * Look in the `puzzle_mount.xacro` file to locate the appropriate `<child_link>` name.  You may need to study its various `<link>` and `<joint>` definitions to find the root link of this part.
    * The `tool0` frame is standardized across most ROS-I URDFs to be the robot's end-effector mounting flange.

 1. Launch the demo.launch file within your moveit_config package to verify the workcell. There should be a grinder sticking out of the table and a puzzle-shaped part attached to the robot.
    ``` bash
    roslaunch myworkcell_moveit_config demo.launch
    ```

### Add new planning group to your moveit_config package.
 1. Re-run the MoveIt! Setup Assistant and create a new Planning Group named **puzzle**.  Define the kinematic chain to extend from the **base_link** to the new **part** link.
    ``` bash
    roslaunch myworkcell_moveit_config setup_assistant.launch 
    ```
    * _Note: Since you added geometry, you should also regenerate the allowed collision matrix._

### Complete Advanced Descartes Node
 1. First, the function `makePuzzleToolPoses()` needs to be completed. The file path for **puzzle_bent.csv** is needed. For portability, don't hardcode the full path. Please use the ROS tool `ros::package::getPath()` to retrieve the root path of the relevant package.
    * reference [getPath()](http://docs.ros.org/melodic/api/roslib/html/c++/namespaceros_1_1package.html#ae9470dd201aa4e66abb833e710d812a4) API
 1. Next, the function `makeDescartesTrajectory()` needs to be completed. The transform between **world** and **grinder_frame** needs to be found. Also Each point needs to have the orientation tolerance set for the z-axis to +/- PI;
    * reference [lookupTransform()](http://docs.ros.org/melodic/api/tf/html/c++/classtf_1_1Transformer.html#ac01a9f8709a828c427f1a5faa0ced42b) API
    * reference [tf::conversions](http://docs.ros.org/melodic/api/tf_conversions/html/c++/tf__eigen_8h.html) namespace
    * reference [TolerancedFrame](https://github.com/ros-industrial-consortium/descartes/blob/melodic-devel/descartes_trajectory/include/descartes_trajectory/cart_trajectory_pt.h#L156) definition
    * reference [OrientationTolerance](https://github.com/ros-industrial-consortium/descartes/blob/melodic-devel/descartes_trajectory/include/descartes_trajectory/cart_trajectory_pt.h#L139) definition

### Update the setup.launch file.
 1. Update the file to take a boolean argument named **adv** so that either the basic or advanced descartes node can be launched.  Use `<if>` and `<unless>` modifiers to control which node is launched.
    * reference [roslaunch XML](http://wiki.ros.org/roslaunch/XML) wiki

### Test Full Application

 1. Run the new setup file, then your main advanced workcell node:

    ``` bash
    roslaunch myworkcell_support setup.launch adv:=true
    rosrun myworkcell_core adv_myworkcell_node
    ```

    * Descartes can take **several minutes** to plan this complex path, so be patient.
    * It's difficult to see what's happening with the rviz planning-loop animation always running.  Disable this loop animation in rviz (Displays -> Planned Path -> Loop Animation) before running `adv_myworkcell_node`.


