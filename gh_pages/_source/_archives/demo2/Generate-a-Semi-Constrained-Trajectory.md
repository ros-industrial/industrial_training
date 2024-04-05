# Generate a Semi-Constrained Trajectory
>In this exercise, we will be creating a Descartes trajectory from an array of cartesian poses.  Each point will have rotational freedom about the z axis of the tool.

## Locate exercise source file

  * Go to the main application source file located in `plan_and_run/src/plan_and_run_node.cpp`.
  * In the main program, locate the function call to `application.generateTrajectory()`. 
  * Go to the source file for that function located in the `plan_and_run/src/tasks/generate_trajectory.cpp`.
     * Alternatively, in QTCreator, click on any part of the function and press "F2" to bring up that file.
  * Comment out the first line containing the `ROS_ERROR_STREAM( ...` entry so that the function does not quit immediately.

## Complete Code
 * Observe how the 'createLemniscate()' is invoked in order to generate all the poses of the tool for the trajectory. Its poses are then used to create the Descartes Trajectory.
 * Use the [AxialSymmetric](http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1AxialSymmetricPt.html#a552cfabcd4891ea01886fa1b258de7f1) constructor to specify a point with rotational freedom about the z-axis.
 * The [AxialSymmetricPt::FreeAxis::Z_AXIS](http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1AxialSymmetricPt.html#a65bf672235bde219db6667892efebbc2) enumeration constant allows you to specify **Z** as the free rotational axis
 * Find comment block that starts with `/*  Fill Code:` and complete as per described .

 * Replace every instance of `[ COMPLETE HERE ]` accordingly.

## Build Code and Run

 * CD into your catkin workspace and run `catkin build`
 * Then run the application launch file
```
roslaunch plan_and_run demo_run.launch
```

## API References

[descartes_trajectory::AxialSymmetricPt](http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1AxialSymmetricPt.html)
