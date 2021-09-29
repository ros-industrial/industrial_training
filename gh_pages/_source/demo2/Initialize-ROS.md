# Initialize ROS
>In this exercise, we will initialize the ros components that our application needs in order to communicate with MoveIt! and other parts of the system.

## Locate Exercise Source File

  * Go to the main application source file located in `plan_and_run/src/plan_and_run_node.cpp`.
  * In the main program, locate the function call to `application.initRos()`. 
  * Go to the source file for that function located in the `plan_and_run/src/tasks/init_ros.cpp`.
     * _Alternatively, in QTCreator, click on any part of the function and press "F2" to bring up that file._
  * Comment out the first line containing the `ROS_ERROR_STREAM ...` entry so that the function does not quit immediately.

## Complete Code

 * Observe how the ros Publisher `marker_publisher_` variable is initialized. The node uses it to publish a `visualization_msgs::!MarkerArray` message for visualizing the trajectory in RViz.
 * Initialize the `moveit_run_path_client_ptr_` action client with the `ExecuteTrajectoryAction`  type.

 * Find comment block that starts with `/*  Fill Code:` and complete as per described.

 * Replace every instance of `[ COMPLETE HERE ]` accordingly.

## Build Code and Run

 * `cd` into your catkin workspace and run `catkin build`
 * Then run the application launch file:
```
roslaunch plan_and_run demo_run.launch
```


## API References

[visualization_msgs::MarkerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html)

[NodeHandle::serviceClient()](http://docs.ros.org/melodic/api/roscpp/html/classros_1_1NodeHandle.html#a183d4cba0ea5c78f075304b91e07cc61)
