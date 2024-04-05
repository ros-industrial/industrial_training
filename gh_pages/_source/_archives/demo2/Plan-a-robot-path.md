# Plan a Robot Path
>In this exercise, we will pass our trajectory to the Descartes planner in order to plan a robot path.

## Locate Exercise Source File

  * Go to the main application source file located in `plan_and_run/src/plan_and_run_node.cpp`.
  * In the main program, locate the function call to `application.planPath()`. 
  * Go to the source file for that function located in the `plan_and_run/src/tasks/plan_path.cpp`. Alternatively, in Eclipse you can click in any part of the function and press "F2" to bring up that file.
  * Comment out the first line containing the `ROS_ERROR_STREAM( ...` entry so that the function does not quit immediately.

## Complete Code

 * Observe the use of the [AxialSymmetricPt::getClosestJointPose()](http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1CartTrajectoryPt.html#a1252c8f49a6e5a7d563b6d4a256b553b) in order to get the joint values of the robot that is closest to an arbitrary joint pose.  Furthermore, this step allows us to select a single joint pose for the start and end rather than multiple valid joint configurations.
 * Call the [DensePlanner::planPath()](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html#a2181f674af57b92023deabb5e8323a2a) method in order to compute a motion plan.
 * When planning succeeds, use the [DensePlanner::getPath()](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html#aafd40b5dc5ed39b4f10e9b47fda0419f) method in order to retrieve the path from the planner and save it into the `output_path` variable.
 * Find comment block that starts with `/*  Fill Code:` and complete as described.

 * Replace every instance of `[ COMPLETE HERE ]` accordingly.

## Build Code and Run

 * `cd` into your catkin workspace and run `catkin build`
 * Then run the application launch file:
```
roslaunch plan_and_run demo_run.launch
```

## API References

[descartes_planner::DensePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html)
