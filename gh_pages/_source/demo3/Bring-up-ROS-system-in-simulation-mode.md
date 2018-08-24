# Start in Simulation Mode
>In this exercise, we will start a ROS system that is ready to move the robot in simulation mode.

## Run setup launch file in simulation mode (simulated robot and sensor)

In a terminal
```
roslaunch demo3 demo3.launch
```

Rviz will display all the workcell components including the robot in its default position; at this point
your system is ready.  However no motion will take place until we run the pick and place node.

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
