# Move Arm to Wait Position
>The '''MoveGroup''' class in '''moveit''' allows us to move the robot in various ways.  With '''MoveGroup''' it is possible to move to a desired joint position, cartesian goal or a predefined pose created with the Setup Assistant.  In this exercise, we will move the robot to a predefined joint pose.

## Locate Function

  * In the main program , locate the method call to '''application.move_to_wait_position()'''.
  * Go to the source file of that function by clicking in any part of the function and pressing "F3".
  * Alternatively, browse to the file in '''[Source directory]/src/tasks/move_to_wait_position.cpp'''.
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

  * The name of the predefined "wait" pose was saved in the global variable '''cfg.WAIT_POSE_NAME''' during initialization.

##  Build Code and Run

  * Compile the pick and place node  in Eclipse
```
Project -> Build Project
```

  * Alternatively, in a terminal cd into the '''demo_manipulation''' directory and do the following
```
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles' --pkg collision_avoidance_pick_and_place
source ./devel/setup.bash
```

  * Run your node with the launch file:
```
roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch
```
  * If the robot is not already in the wait position, it should move to the wait position. In the terminal, you will see something like the following message:
```
[ INFO] [1400553673.460328538]: Move wait Succeeded
[ERROR] [1400553673.460434627]: set_gripper is not implemented yet.  Aborting.
```

## API References

[setNamedTarget()](http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#af6850334bb1b4f12e457257550d5f92c)

[move()](http://docs.ros.org/hydro/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#a4c63625e2e9eb5c342d1fc6732bd8cf7)
