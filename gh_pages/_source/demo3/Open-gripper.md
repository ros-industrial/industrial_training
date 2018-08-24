# TODO Update this for hardware
# Open Gripper
>In this exercise, the objective is to use a "grasp action client" to send a grasp goal that will open the gripper.

## Locate Function

  * In the main program , locate the function call to '''application.set_gripper()'''.
  * Go to the source file of that function by clicking in any part of the function and pressing "F3".
  * Remove the fist line containing the following '''ROS_ERROR_STREAM ...''' so that the program runs.


## Complete Code

  * Find every line that begins with the comment "''Fill Code: ''" and read the description.  Then, replace every instance of the comment  "''ENTER CODE HERE''"
 with the appropriate line of code.
```
/* Fill Code:
     .
     .
     .
*/
/* ========  ENTER CODE HERE ======== */
```

 * The 'grasp_goal' goal property can take on three possible values:
```
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP;
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE;
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP;
```

  * Once the grasp flag has been set you can send the goal through the grasp action client


## Build Code and Run

  * Compile the pick and place node in QT
```
Project -> Build Project
```

  * Alternatively, in a terminal cd into the '''demo_manipulation''' directory and do the following
```
catkin build collision_avoidance_pick_and_place
```

  * Run your node with the launch file:
```
roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch
```

  * If the task succeeds you will see something like the following in the terminal (below). The robot will not move, only gripper I/O is triggered:
```
[ INFO] [1400553290.464877904]: Move wait Succeeded
[ INFO] [1400553290.720864559]: Gripper opened
[ERROR] [1400553290.720985315]: detect_box_pick is not implemented yet.  Aborting.
```

## API References

[sendGoal()](http://ros.org/doc/hydro/api/actionlib/html/classactionlib_1_1SimpleActionClient.html)
