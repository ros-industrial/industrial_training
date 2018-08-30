# Create Pick and Place Helpers 
With knowledge of the way that TrajOpt works, we will now use it to build our application. Consider the motion steps in a pick and place operation.

***Pick***

* Free space move to approach pose
* Linear move to target pose

***Place*** 

* Linear move to retreat pose
* Free space move to approach pose
* Linear move to target pose

In order to simplify the scripting of the pick and place operation, helper functions are defined. These are then used in ```test_bed_core_node``` to perform the operation.

## Explore function definitions

  * In pick_and_place/include open trajopt_pick_and_place_constructor.h
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

You may find Sections 3.5 and 3.6 helpful in completing this exercise. Additionally, QT Creator's autocomplete functionality can aid in finding the correct methods described in the comments.

## Build Code and Run

The code can be run by using the same launch file as before. This time the robot should pathplan when the perception service completes

```roslaunch pick_and_place pick_and_place.launch```
