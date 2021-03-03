# Motion Planning using C++
>In this exercise, we'll explore MoveIt's C++ interface to programatically move a robot. 


## Motivation
Now that we’ve got a working MoveIt! configuration for your workcell and we’ve played a bit in RViz with the planning tools, let’s perform planning and motion in code. This exercise will introduce you to the basic C++ interface for interacting with the MoveIt! node in your own program. There are lots of ways to use MoveIt!, but for simple applications this is the most straight forward.

## Reference Example
[Move Group Interface tutorial](http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html#setup)

## 3. Further Information and Resources
 * [MoveIt! Tutorials](http://docs.ros.org/kinetic/api/moveit_tutorials/html/)
 * [MoveIt! home-page](http://moveit.ros.org/)

## Scan-N-Plan Application: Problem Statement
In this exercise, your goal is to modify the `myworkcell_core` node to:

 1. Move the robot’s tool frame to the center of the part location as reported by the service call to your vision node.

## Scan-N-Plan Application: Guidance

 1. Edit your `myworkcell_node.cpp` file.

    1. Add `#include <tf/tf.h>` to allow access to the tf library (for frame transforms/utilities).

       * Remember that we already added a dependency on the `tf` package in a previous exercise.

    1. In the `ScanNPlan` class's `start` method, use the response from the `LocalizePart` service to initialize a new `move_target` variable:

       ``` c++
       geometry_msgs::Pose move_target = srv.response.pose;
       ```

       * make sure to place this code _after_ the call to the vision_node's service.

 1. Use the `MoveGroupInterface` to plan/execute a move to the `move_target` position:

    1. In order to use the `MoveGroupInterface` class it is necessary to add the `moveit_ros_planning_interface` package as a dependency of your `myworkcell_core` package. Add the `moveit_ros_planning_interface` dependency by modifying your package's `CMakeLists.txt` (2 lines) and `package.xml` (1 line) as in previous exercises.

    1. Add the appropriate "include" reference to allow use of the `MoveGroupInterface`:
      
       ```c++
       #include <moveit/move_group_interface/move_group_interface.h>
       ``` 

    1. Create a `moveit::planning_interface::MoveGroupInterface` object in the `ScanNPlan` class's `start()` method. It has a single constructor that takes the name of the planning group you defined when creating the workcell moveit package (“manipulator”).

       ```c++
       moveit::planning_interface::MoveGroupInterface move_group("manipulator");
       ```

    1. Set the desired cartesian target position using the `move_group` object’s `setPoseTarget` function. Call the object's `move()` function to plan and execute a move to the target position.

       ```c++
       // Plan for robot to move to part
       move_group.setPoseReferenceFrame(base_frame);
       move_group.setPoseTarget(move_target); 
       move_group.move();
       ```

    1. As described [here](http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#a4c63625e2e9eb5c342d1fc6732bd8cf7), the `move_group.move()` command requires use of an "asynchronous" spinner, to allow processing of ROS messages during the blocking `move()` command.  Initialize the spinner near the start of the `main()` routine after `ros::init(argc, argv, "myworkcell_node")`, and **replace** the existing `ros::spin()` command with `ros::waitForShutdown()`, as shown:

       ```c++
       ros::AsyncSpinner async_spinner(1);
       async_spinner.start();
       ...
       ros::waitForShutdown();
       ```

 1. Test the system!

    ``` bash
    catkin build
    roslaunch myworkcell_moveit_config myworkcell_planning_execution.launch
    roslaunch myworkcell_support workcell.launch
    ```

 1. More to explore...
    * In RViz, add a "Marker" display of topic "/ar_pose_visual" to confirm that the final robot position matches the position published by `fake_ar_publisher`
    * Try repeating the motion planning sequence:
      1. Use the MoveIt rviz interface to move the arm back to the "allZeros" position
      1. Ctrl+C the `workcell.launch` file, then rerun
    * Try updating the `workcell_node`'s `start` method to automatically move back to the `allZeros` position after moving to the AR_target position.  See [here](http://docs.ros.org/kinetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html) for a list of `move_group`'s available methods.
    * Try moving to an "approach position" located a few inches away from the target position, prior to the final move-to-target.
