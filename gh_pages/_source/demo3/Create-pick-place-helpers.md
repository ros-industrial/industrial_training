# Create Pick and Place Helpers 
> Consider the steps in a pick and place operation.

***Pick***

* Free space move to approach pose
* Linear move to target pose

***Place*** 

* Linear move to retreat pose
* Free space move to approach pose
* Linear move to target pose

In order to simplify the scripting of the pick and place operation, helper functions are defined. These are then used in ```test_bed_core_node``` to perform the operation.

## Explore function definitions

  * In demo3/include open trajopt_pick_and_place_constructor.h
  * Look at each of public member functions. These are the functions you will complete. Understand each the inputs and purpose of each of them.

## Complete Code

  * Open trajopt_pick_and_place_constructor.cpp
  * Find every line that begins with the comment "''Fill Code: ''" and read the description.  Then, replace every instance of the comment  "''ENTER CODE HERE''"
 with the appropriate line of code
```
/* Fill Code:
     .
     .
     .
*/
// ENTER CODE HERE: 
```


 * The [[lookupTransform|http://mirror.umd.edu/roswiki/doc/hydro/api/tf/html/c++/classtf_1_1Transformer.html#ac01a9f8709a828c427f1a5faa0ced42b]] method can provide the pose of a target relative to another pose.

## Build Code and Run

The code can be run by using the same launch file as before. This time the robot should pathplan when the perception service completes

```roslaunch pick_and_place pick_and_place.launch```
