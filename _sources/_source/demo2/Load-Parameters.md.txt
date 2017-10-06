# Load Parameters
>In this exercise, we'll load some ROS parameters to initialize important variables within our program.

## Locate Exercise Source File

  * Go to the main application source file located in `plan_and_run/src/plan_and_run_node.cpp`.
  * In the main program, locate the function call to `application.loadParameters()`. 
  * Go to the source file for that function located in the `plan_and_run/src/tasks/load_parameters.cpp`. Alternatively, in Eclipse you can click in any part of the function and press "F3" to bring up that file.
  * Comment out the first line containing the `//ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);` entry so that the function doesn't quit immediately.

## Complete Code

 * Find comment block that starts with ```/*  Fill Code:``` and complete as described.

 * Replace every instance of ```[ COMPLETE HERE ]``` accordingly.

## Build Code and Run

 * `cd` into your catkin workspace and run:
```
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles'
source ./devel/setup.bash
```
 * Then run the application launch file:
```
roslaunch plan_and_run demo_setup.launch
roslaunch plan_and_run demo_run.launch
```

## API References

[ros::NodeHandle](http://docs.ros.org/indigo/api/roscpp/html/classros_1_1NodeHandle.html)

[NodeHandle::getParam()](http://docs.ros.org/indigo/api/roscpp/html/classros_1_1NodeHandle.html#afaaf745b7483da9a621b07db0700f866)
