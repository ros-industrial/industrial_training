# Initialize Descartes
>This exercise consist of setting up the Descartes Robot Model and Path Planner that our node will use to plan a path from a semi-constrained trajectory of the tool.

## Locate Exercise Source File

  * Go to the main application source file located in '''plan_and_run/src/plan_and_run_node.cpp'''.
  * In the main program , locate the function call to '''application.initDescartes()'''. 
  * Go to the source file for that function located in the '''plan_and_run/src/tasks/init_descartes.cpp'''. Alternatively, in Eclipse you can click in any part of the function and press "F3" to bring up that file.
  * Comment out the first line containing the ```//ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);``` entry so that the function doesn't quit immediately.

## Complete Code

 * Invoke the [[descartes_core::RobotModel::initialize()|http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html#af6e9db3c1dec85046fc836136cf7b0fb]] method in order to properly initialize the robot.
 * Similarly, initialize the Descartes planner by passing the ''robot_model_'' variable into the '''descartes_core::!DensePlanner::initialize()''' method.
 * Find comment block that starts with ```/*  Fill Code:``` and complete as described.

 * Replace every instance of ```[ COMPLETE HERE ]``` accordingly.

## Build Code and Run

 * `cd` into your catkin workspace and run `catkin build`
 * Then run the application launch file:
```
roslaunch plan_and_run demo_run.launch
```


## API References

[descartes_core::RobotModel](http://docs.ros.org/indigo/api/descartes_core/html/classdescartes__core_1_1RobotModel.html)
[descartes_planner::DensePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html)
