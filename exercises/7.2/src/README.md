
ROS2 Descartes Demo
---
### Build the Demo
This exercise uses the ROS1 bridge to call ROS nodes from  ROS2 nodes and therefore the build procedure is somewhat involved and the following steps describe how to go about it
#### 1 -  Create a ROS workspace for exercise 4.1
1. Clone the ROS-I training repo in an accessible location
```
git clone https://github.com/ros-industrial/industrial_training.git
```
1. Create a catkin workspace for the exercise 4.1 ROS packages and dependencies
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```
1. Create a soft link to exercise 4.1 in the training repo
```
cd ~/catkin_ws/src
ln -s ~/industrial_training/exercises/4.1/src/ demo
```
1. Clone additional dependencies
```
git clone https://github.com/ros-industrial-consortium/descartes.git
git clone https://github.com/ros-industrial/universal_robot.git
git clone https://github.com/Jmeyer1292/fake_ar_publisher.git
```
1. Source and build exercise
```
. /opt/ros/melodic/setup.bash
catkin build --jobs=4
```

#### 2 -  Create the ROS2 workspace

1. Create a colcon workspace for the exercise 7.2 ROS packages and dependencies
```
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
```
1. Create a soft link to exercise 7.2
```
ln -s ~/industrial_training/exercises/7.2/src demo
```
1. Source and build
```
cd ..
. /opt/ros/eloquent/setup.bash
colcon build
```

#### 3 -  Create the ROS1 Bridge ROS2 workspace
Since this exercise uses custom services then it is necessary to build the bridge in order to register these services with it.
1. Create the ros1_bridge workspace
```
mkdir -p ~/ros1_bridge_ws/src
cd ~/ros1_bridge_ws/src
```
1. Clone and select the branch that matches your ros release
```
git clone https://github.com/ros2/ros1_bridge.git
cd ros1_bridge/
git checkout eloquent
```
1. Source ROS and ROS2 setup bash files
```
cd ~/ros1_bridge_ws/
. /opt/ros/melodic/setup.bash
. /opt/ros/eloquent/setup.bash
. ~/catkin_ws/devel/setup.bash
. ~/colcon_ws/install/local_setup.bash
```
1. Build the bridge
```
colcon build --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=FALSE
```
1. List the mapped message and service pairs
```
source install/setup.bash
ros2 run ros1_bridge dynamic_bridge --print-pairs
```
This should list the custom `myworkcell_msgs(ROS2) <-> myworkcell_core` mapped services

---
### Run the Demo

#### 1 - Run the ROS nodes
1. In terminal sourced to your ROS catkin_ws workspace start the roscore
```
roscore
```
2. In another sourced terminal, run the following launch file:
```
roslaunch myworkcell_support ros2_setup.launch
```
This will launch the motion planning and motion execution nodes is ROS.  Rviz will come up as well.

#### 2 - Run the ROS1 bridge
1. In your ros1_bridge_ws ROS2 workspace, source the workspace if you haven't
```
cd ~/ros1_bridge_ws
source install/setup.bash
```
1. export the _ROS_MASTER_URI_ environment variable
```
export ROS_MASTER_URI=http://localhost:11311
```
1. Run the bridge
```
ros2 run ros1_bridge dynamic_bridge
```

#### 2 - Run the ROS2 nodes
1. Open a new terminal and source your ROS2 workspace
```
cd ~/colcon_ws
source install/setup.bash
```
1. Run the python launch file that starts the ROS2 nodes for this application
```
ros2 launch myworkcell_core workcell.launch.py
```
If the program succeeds you should see the following output:
> 
[myworkcell_node-2] Got base_frame parameter world
[myworkcell_node-2] Waiting for client /localize_part
[myworkcell_node-2] Waiting for client /plan_path
[myworkcell_node-2] Waiting for client /move_to_pose
[myworkcell_node-2] Waiting for client /execute_trajectory
[myworkcell_node-2] Found all services
[myworkcell_node-2] Requesting pose in base frame: world
[myworkcell_node-2] Part localized
[myworkcell_node-2] Planning trajectory
[myworkcell_node-2] Executing trajectory
[myworkcell_node-2] Trajectory execution complete

You should also see the robot moving accordingly in the Rviz window.

---
#### Issues:
1. 
**Problem**: The application only runs successfully once, the next run will fail due to a tf lookup operation in the vision_node.  This likely happens due to the bridge removing the bridging for all `/tf` topics after the application ends.
Solution:
**Solution**: Restart the `ros1_bridge` and then the `workcell.launch.py` application.


