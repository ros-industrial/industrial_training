# Run a Robot Path
>In this exercise, we'll convert our Descartes path into a MoveIt! trajectory and then send it to the robot.

## Locate Exercise Source File

  * Go to the main application source file located in '''plan_and_run/src/plan_and_run_node.cpp'''.
  * In the main program , locate the function call to '''application.runPath()'''. 
  * Go to the source file for that function located in the '''plan_and_run/src/tasks/run_path.cpp'''. Alternatively, in Eclipse you can click in any part of the function and press "F3" to bring up that file.
  * Comment out the first line containing the ```//ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);``` entry so that the function doesn't quit immediately.

## Complete Code

 * Find comment block that starts with ```/*  Fill Code:``` and complete as described.

 * Replace every instance of ```[ COMPLETE HERE ]``` accordingly.

## Build Code and Run

 * `cd` into your catkin workspace and run `catkin build`
 * Then run the application launch file:
```
roslaunch plan_and_run demo_run.launch
```

## API References

[MoveGroupInterface::move()](http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#a4c63625e2e9eb5c342d1fc6732bd8cf7)
