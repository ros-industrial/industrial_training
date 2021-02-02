# ROS2-Cartographer

## 1. Introduction
- The goal of this tutorial is to
    - use Cartographer to create a map of environment

- The packages that will be used:
    - cartographer 
    - cartographer-ros
    - turtlebot3_cartographer
    - turtlebot3_teleop
    - turtlebot3_gazebo

This tutorial explains how to use the Cartographer for mapping and localization. 
* Lines beginning with `$` indicates the syntax of these commands.
Commands are executed in a terminal:
    * Open a new terminal → use the shortcut ctrl+alt+t.
Open a new tab inside an existing terminal → use the shortcut ctrl+shift+t.

* Info: The computer of the real robot will be accessed from your local computer remotely. For every further command, a tag will inform which computer has to be used. It can be either `[TurtleBot]` or `[Remote PC]`.


## 2. General approach
The main problem in mobile robotics is localization and mapping. To estimate the position of the robot in an environment, you need some kind of map from this environment to determine the actual position in this environment. On the other hand, you need the actual position of robots to create a map related to its position. Therefore you can use SLAM – Simultaneous Localization and Mapping. ROS provides different
packages to solve this problem:
* **2D**: gmapping, hector_slam, cartographer, ohm_tsd_slam...
* **3D**: rgbdslam, ccny_rgbd, lsd_slam, rtabmap...

For ground-based robots, it is often sufficient to use 2D SLAM to navigate through the environment. In the following tutorial, cartographer will be used. Cartographer SLAM builds a map of the environment and simultaneously estimates the platform's 2D pose. The localization is based on a laser scan and the odometry of the robot. 



## 3. Start Cartographer
### 3.1 Technical Overview
![Technical Overview](../../_static/high_level_system_overview.png)

Figure 1: Technical Overview

source: [cartographer](https://google-cartographer.readthedocs.io/en/latest/)



### 3.2 Install cartographer

Install cartographer by performing the following:

**Before installing package, you need to make sure which ROS  Distro you are using.**

After sourcing ROS2, you will get $ROS_DISTRO in evironment values.

```bash
# source ROS2
$ source /opt/ros/foxy/setup.bash
$ echo $ROS_DISTRO
You will get "foxy"
```

```bash
$ sudo apt install ros-$ROS_DISTRO-cartographer ros-$ROS_DISTRO-cartographer-ros  
```
#### Check if there is turtlebot3* package

```bash
$ ros2 pkg list | grep turtlebot3
```
if not you can install these package by performing the following:

First entering your workspace

```bash
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/foxy-devel/turtlebot3.repos
```

Make sure you have "src" folder, then run this command to get source code for turtlebot3

```bash
$ vcs import src<turtlebot3.repos
```
Source your ROS2 installation workspace and install dependencies

```bash
$ source /opt/ros/foxy/setup.bash
$ rosdep update
$ rosdep install --from-paths src --ignore-src --rosdistro 
$ROS_DISTRO -y
```
Compile codes

```bash
$ colcon build
```
Source your workspace
```bash
$ source install/setup.bash
```


### 3.3 Startup system of turtleBot and teleoperation

**Please try simulation first**

#### 3.3.1 Simulation

a. Simulation in gazebo

Here we use 11 as ROS Domain ID

1. Set up ROS_DOMAIN_ID

    ```bash
    $ export ROS_DOMAIN_ID=11
    ```

2. Set up turtlebot model

    ```bash
    $ export TURTLEBOT3_MODEL=burger
    ```

3. Set up Gazebo model path

    ```bash
    $ export GAZEBO_MODEL_PATH=`ros2 pkg \
    prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/
    ```

4. Launch Gazebo with a simulation world

    ```bash
    $ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```


#### 3.3.2 Real robot

[TurtleBot3]

a. open a terminal and use `ssh` connect to Turtlebot3.

b. bring up basic packages to start its applications.
```bash
$ source .bashrc
$ cd turtlebot3_ws
$ source install/setup.bash
$ ros2 launch turtlebot3_bringup robot.launch.py
```

#### 3.3.3 Run teleoperation node

[Remote PC]

1. Open a new terminal

2. Make sure you are in the workspace which you created at the beginning.

3. Then source your workspace.

    ```bash
    $ source install/setup.bash
    ```

4. Set up ROS_DOMAIN_ID

    ```bash
    $ export ROS_DOMAIN_ID=11
    ```

5. Set up turlebot model

    ```bash
    $ export TURTLEBOT3_MODEL=burger
    ```
6. Run teleoperation node

    ```bash
    $ ros2 run turtlebot3_teleop teleop_keyboard
    ```


### 3.4 Run SLAM nodes

1. open a new terminal on Remote PC and enter your workspace

2. source your workspace

3. export ROS domain id which is same as the one for TurtleBot

    `[Remote PC]`

    ```bash
    $ export ROS_DOMAIN_ID=11
    ```

4. run the SLAM nodes

    If you are using simulation, you need to use simulation time. You can set `use_sim_time` to True.

    **a. Simulation**

    `[Remote PC]`

    ```bash
    $ ros2 launch turtlebot3_cartographer \
    cartographer.launch.py \
    use_sim_time:=True
    ```

    **b. For a real robot**

    `[Remote PC]`

    ```bash
    $ ros2 run cartographer_ros occupancy_grid_node -resolution 0.05\
    -publish_period_sec 1.0

    $ ros2 run cartographer_ros cartographer_node\
    -configuration_directory]\ 
    install/turtlebot3_cartographer/share/turtlebot3_cartographer\
    /config -configuration_basename turtlebot3_lds_2d.lua
    ```


 1. create a map
    
    ***Hint: Make sure that the Fixed Frame (in Global Options) in RViz is set to “map”.*** 

    In this way the map is fixed and the robot will move relative to it.
    The scanner of the Turtlebot3 covers 360 degrees of its surroundings. Thus, if objects are close by to the robot it will start to generate the map. 

    Teleoperate the robot through the physical world until the enclosed environment is completely covered in the virtual map.

    The following hints help you to create **a nice map**:

        * Try to drive as slow as possible
        * Avoid to drive linear and rotate at the same time
        * Do not drive too close to the obstacles

    In the left menu of RViz you can see several display modules. There is e.g. the RobotModel which is virtual visualization of the robot. 

    Furthermore, you can visualize the transforms of the available frames by checking the box of tf. Make yourself familiar with the available modules.
    
    ![Map at startup](../../_static/beforemap.png)
    Figure 2: A incomplete map at beginning in the real work setup

    ![Final Map](../../_static/aftermap2.png)
    Figure 3: A complete map at beginning in the real work setup



## 4. Save the Map

If you are satisfied with your map you can store it. Open a new terminal and run the map saver node.

`[Remote PC]`

First you need to source workspace and set up ROS Domain ID that is same as before.
```
$ ros2 run nav2_map_server map_saver_cli 
```
You also can define a name of the map by
```
ros2 run nav2_map_server map_saver_cli -f my_map
```
If the terminal’s path is "your workspace" they can be found in "your workspace" directory. An example of the map.pgm image is given in the following.

![Map](../../_static/map.jpg)


The node will create a map.pgm and a map.yaml files in the current directory, which is in this case the home directory.

***Hint: The signs “~/” is a direct path to the home directory which works from every relative path.***
