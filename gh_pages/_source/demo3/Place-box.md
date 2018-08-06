# Place Box
>In this exercise, we will move the robot through the place motions while avoiding obstacles with an attached payload.  In addition, the gripper must be opened or close at the appropriate time in order to complete the task.

## Locate Function

  * In the main program , locate the function call to '''application.place_box()'''.
  * Go to the source file of that function by clicking in any part of the function and pressing "F3".
  * Remove the fist line containing the following '''ROS_ERROR_STREAM ...''' so that the program runs.

## Complete Code

  * Find every line that begins with the comment "''Fill Code: ''" and read the description.  Then, replace every instance of the comment  "''ENTER CODE HERE''"
 with the appropriate line of code
```
/* Fill Code:
     .
     .
     .
*/
/* ========  ENTER CODE HERE ======== */
```

 * The [[execute()|http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#a82f1bb33058893e8a16fa49af24d689f]] method sends a motion plan to the robot.

## Build Code and Run

  * Compile the pick and place node  in Eclipse
```
Project -> Build Project
```
  * Alternatively, in a terminal cd into the '''demo_manipulation''' directory and do the following
```
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles' --pkg collision_avoidance_pick_and_place
```

  * Run your node with the launch file:
```
roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch
```
  * At this point your exercise is complete and the robot should move through the pick and place motions and then back to the wait pose. Congratulations!

## API References

[Useful " MoveGroup " Methods](http://ros.org/rosdoclite/groovy/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html)
