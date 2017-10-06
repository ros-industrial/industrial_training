# Initialize ROS
>In this exercise, we'll initialize the ros components that our application needs in order to communicate to MoveIt! and other parts of the system.

## Locate Exercise Source File

  * Go to the main application source file located in '''plan_and_run/src/plan_and_run_node.cpp'''.
  * In the main program , locate the function call to '''application.initRos()'''. 
  * Go to the source file for that function located in the '''plan_and_run/src/tasks/init_ros.cpp'''. Alternatively, in Eclipse you can click in any part of the function and press "F3" to bring up that file.
  * Comment out the first line containing the ```//ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);``` entry so that the function doesn't quit immediately.

## Complete Code

 * Observe how the ros Publisher '''marker_publisher_''' variable is initialized.  The node uses it to publish a ''visualization_msgs::!MarkerArray'' message for visualizing the trajectory in RViz.
 * Initialize the '''moveit_run_path_client_''' service client using the ros node handle's '''ros::!NodeHandle::serviceClient()''' templated method.

 * Find comment block that starts with ```/*  Fill Code:``` and complete as described.

 * Replace every instance of ```[ COMPLETE HERE ]``` accordingly.

## Build Code and Run

 * `cd` into your catkin workspace and run `catkin build`
 * Then run the application launch file:
```
roslaunch plan_and_run demo_run.launch
```


## API References

[visualization_msgs::MarkerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html)

[NodeHandle::serviceClient()](http://docs.ros.org/indigo/api/roscpp/html/classros_1_1NodeHandle.html#aa3376eeca609c4985255cecfaadcbcc5)
