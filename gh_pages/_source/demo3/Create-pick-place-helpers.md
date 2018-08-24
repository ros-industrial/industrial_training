# Create Pick and Place Helpers 
> Consider the steps in a pick and place operation.

***Pick***

* Free space move to approach pose
* Linear move to target pose

***Place*** 

* Linear move to retreat pose
* Free space move to approach pose
* Linear move to target pose

In order to simplify the scripting of the pick and place operation, helper functions are defined.

## Explore function definitions

  * In demo3/include open trajopt_pick_and_place_constructor.h
  * Look at each of public member functions. These are the functions you will complete. Understand each the inputs and purpose of each of them.

## Complete Code

  * Open trajopt_pick_and_place_constructor.cpp
  * Find every line that begins with the comment "''Fill Code: ''" and read the description.  Then, replace every instance of the comment  "''ENTER CODE HERE''"
 with the appropriate section of code
```
/* Fill Code:
     .
     .
     .
*/
/* ========  ENTER CODE HERE ======== */
```

 * The '''create_manipulation_poses()''' uses the values of the approach and retreat distances in order to create the corresponding poses at the desired target.

 * Since moveit plans the robot path for the arm's wrist, then it is necessary to convert all the pick poses to the wrist coordinate frame.

 * The [[lookupTransform|http://mirror.umd.edu/roswiki/doc/hydro/api/tf/html/c++/classtf_1_1Transformer.html#ac01a9f8709a828c427f1a5faa0ced42b]] method can provide the pose of a target relative to another pose.

## Build Code and Run


