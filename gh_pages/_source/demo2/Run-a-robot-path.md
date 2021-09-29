# Run a Robot Path
>In this exercise, we will convert our Descartes path into a MoveIt! trajectory and then send it to the robot.

## Locate Exercise Source File

  * Go to the main application source file located in `plan_and_run/src/plan_and_run_node.cpp`.
  * In the main program, locate the function call to `application.runPath()`. 
  * Go to the source file for that function located in the `plan_and_run/src/tasks/run_path.cpp`.
     * Alternatively, in QTCreator, click on any part of the function and press "F2" to bring up that file.
  * Comment out the first line containing the `ROS_ERROR_STREAM( ...` entry so that the function does not quit immediately.

## Complete Code

 * Find comment block that starts with `/*  Fill Code:` and complete as per described.

 * Replace every instance of `[ COMPLETE HERE ]` accordingly.

## Build Code and Run

 * `cd` into your catkin workspace and run `catkin build`
 * Then run the application launch file:
```
roslaunch plan_and_run demo_run.launch
```

## API References

[MoveGroupInterface::move()](http://docs.ros.org/melodic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html#a3513c41b0c73400fc6713b25bc6b1637)
