# Inspect the "pick_and_place" Package
In this exercise, we will get familiar with all the files that you'll be interacting with throughout these exercises. 

## Acquire and initialize the workspace
```
cp -r ~/industrial_training/exercises/Optimization_Based_Planning/template_ws ~/optimized_planning_ws
cd ~/optimized_planning_ws
source /opt/ros/kinetic/setup.bash
catkin init
```

## Download source dependencies
Use the [wstool](http://wiki.ros.org/wstool) command to download the repositories listed in the **src/.rosinstall** file
```
cd ~/optimized_planning_ws/src/
wstool update
```

## Download debian dependencies
Make sure you have installed and configured the [rosdep tool](http://wiki.ros.org/rosdep).
Then, run the following command from the **/src** directory of your workspace.
```
rosdep install --from-paths . --ignore-src -y -r
```

We also need to install glfw.
``` 
sudo apt install libglfw3-dev libglfw3 
```
## Build your workspace
```
catkin build
```
If the build fails then revisit the previous two steps to make sure all the dependencies were downloaded.


## Source the workspace
Run the following command from your workspace parent directory
```
source devel/setup.bash
```

## Explore your workspace
Your workspace contains 7 packages

* ***pick_and_place*** - This is the main pick and place package. It contains the main pick and place node and the launch file to bring up the system.
* ***pick_and_place_perception*** - This package contains the perception pipeline. We will develop a PCL algorithm to use a 3d camera to detect a pick object and pass it's location to the pick and place node. 
* ***pick_and_place_support*** - This package contains the support files for the system. This is where files such as the robot model are stored. It also contains code for camera calibration.
* ***gl_depth_sim*** - This package simulates a 3d camera by using OpenGL to convert a mesh and a camera pose into a point cloud. It was installed from github by wstool.
* ***tesseract*** - This package runs the planning environment. It contains tools for robot path planning, collision checking, and visualization. It was installed from github by wstool.
* ***trajopt*** - This package contains the trajopt motion planner. We will use this to path plan - optimizing for avoiding collisions, following a set of desired waypoints, and meeting physical robot constraints. It was installed from github by wstool.
* ***iiwa_stack*** - This package contains the hardware driver for controlling a KUKA iiwa collaborative robot. It also contains the robot description that is used for simulation and path planning. It was installed from github by wstool.

## Look into the src directory
Looking in the src directory of the main pick_and_place package, you see the following.

### Nodes:

* **test_bed_core_node.cpp** : Main application thread. Contains all necessary headers and function calls to perform a scripted pick and place operation.

* **sensor_simulator_3d.cpp** : Simulates data from a 3d camera sensor. Only used in simulation.

### Utilities
* **trajopt_pick_and_place_constructor.cpp** : Contains source code for trajopt pick and place helper functions. This is where the majority of the trajopt implementation will be.








