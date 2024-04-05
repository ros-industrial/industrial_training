# Start in Simulation Mode
In this exercise, we will start a ROS system that is ready to move the robot in simulation mode.

## Run setup launch file in simulation mode (simulated robot and sensor)

In a terminal
```
roslaunch pick_and_place pick_and_place.launch
```

Rviz will display all the workcell components including the robot in its default position. 

Look into ```test_bed_core_node.cpp```. This code is already complete. A service to find the pick target is being run, and a TrajOpt problem is set up to perform a pick and place operation. While the service to find the pick target is running, it will not find a pick and the path planning will not run. These will be the next tasks.

