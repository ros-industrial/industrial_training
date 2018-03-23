# Create Place Moves
>The gripper moves through three poses in order to place the box: Approach, place and retreat. In this exercise, we will create these place poses for the '''tcp'''  coordinate frame and then transform them to the arm's wrist coordinate frame.

## Locate Function

  * In the main program , locate the function call to '''application.create_place_moves()'''.
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

 * The box's position at the place location is saved in the global variable '''cfg.BOX_PLACE_TF'''.

 * The '''create_manipulation_poses()''' uses the values of the approach and retreat distances in order to create the corresponding poses at the desired target.

 * Since moveit plans the robot path for the arm's wrist, then it is necessary to convert all the place poses to the wrist coordinate frame.

 * The [[lookupTransform|http://mirror.umd.edu/roswiki/doc/hydro/api/tf/html/c++/classtf_1_1Transformer.html#ac01a9f8709a828c427f1a5faa0ced42b]] method can provide the pose of a target relative to another pose.

## Build Code and Run

  * Compile the pick and place node  in Eclipse
```
Project -> Build Project
```

  * Alternatively, in a terminal cd into the '''demo_manipulation''' directory and do the following
```
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles' --workspace collision_avoidance_pick_and_place
```

  * Run your node with the launch file:
```
roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch
```
  * The tcp and wrist position at the place location will be printed on the terminal. You should see something like:
```
[ INFO] [1400556479.404133995]: Execution completed: SUCCEEDED
[ INFO] [1400556479.404574973]: Pick Move 2 Succeeded
[ INFO] [1400556479.404866351]: tcp position at place: 0x7fff1055d800
[ INFO] [1400556479.404934796]: wrist position at place: x: -0.422
y: 0.6
z: 0.3

[ERROR] [1400556479.404981729]: place_box is not implemented yet.  Aborting.
```



## API References

[lookupTransform](http://mirror.umd.edu/roswiki/doc/hydro/api/tf/html/c++/classtf_1_1Transformer.html#ac01a9f8709a828c427f1a5faa0ced42b)

[TF Transforms and other useful data types](http://wiki.ros.org/tf/Overview/Data%20Types)
