# Start in Simulation Mode
In this exercise, we will start a ROS system that is ready to move the robot in simulation mode.

## Run setup launch file in simulation mode (simulated robot and sensor)

In a terminal
```
roslaunch pick_and_place pick_and_place.launch
```

Rviz will display all the workcell components including the robot in its default position. 

Look in ```test_bed_core_node.cpp```. This code is already complete. A service to find the pick target is run, and a TrajOpt problem is setup to perform a pick and place operation. while, the service to find the pick target will run. it will not find a pick and the pathplanning will not run. These will be the next tasks.

## Setup for real sensor and simulated robot
```
roslaunch demo3 demo3.launch sim_sensor:=false
```

## Setup for real robot and simulated sensor data
```
roslaunch demo3 demo3.launch sim_robot:=false robot_ip:= [robot ip]
```

## Setup for real robot and real sensor
```
roslaunch demo3 demo3.launch sim_robot:=false robot_ip:= [robot ip] sim_sensor:=false sim_gripper:=false
```
