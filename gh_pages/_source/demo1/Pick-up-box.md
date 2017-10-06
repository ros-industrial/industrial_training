# Pick Up Box
>In this exercise, we will move the robot through the pick motion while avoiding obstacles in the environment.  This is to be accomplished by planning for each pose and closing or opening the vacuum gripper when apropriate. Also, it will be demonstrated how to create a motion plan that MoveIt! can understand and solve.

## Locate Function

  * In the main program, locate the function call to '''application.pickup_box()'''.
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

 * Inspect the '''set_attached_object''' method to understand how to manipulate a '''robot_state''' object which will then be used to construct a motion plan.

 * Inspect the '''create_motion_plan''' method to see how an entire motion plan request is defined and sent.

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
  * The robot should go through the pick moves (Approach, pick and retreat) in addition to the moves from the previous exercises. In the terminal you will see something like:
```
[ INFO] [1400555978.404435764]: Execution completed: SUCCEEDED
[ INFO] [1400555978.404919764]: Pick Move 2 Succeeded
[ERROR] [1400555978.405061541]: create_place_moves is not implemented yet.  Aborting.
```

## API References

[Useful '''MoveGroup'''](http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html)
