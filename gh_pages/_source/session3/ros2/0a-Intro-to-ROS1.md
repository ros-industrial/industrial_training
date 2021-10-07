# Introduction to ROS1
>In this exercise, we will explore some basic ROS1 operations and learn how to use the ROS1/2 bridge node.


## Motivation
While the ROS community is focusing most new development on ROS2, there remain many packages that have not yet been updated from ROS1.  Many projects will need to use a combination of ROS1 and ROS2 packages, so it is useful to understand how to work with legacy ROS1 packages.
In addition, ROS1 and ROS2 systems can't directly communicate with each other.  The ROS1 bridge package was created to solve this communication gap.  Understanding how to use this bridge is key to working with hybrid ROS1/2 systems.

## Reference Example

[ROS1 Bridge](https://github.com/ros2/ros1_bridge)

## Further Information and Resources

* [ROS1 Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)
* [ROS1 Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)
* [ROS1 Launch Cmds](http://wiki.ros.org/roslaunch/Commandline%20Tools)
* [ROS1 Launch XML](http://wiki.ros.org/roslaunch/XML)

## Scan-N-Plan Application: Problem Statement
This exercise is not part of the main "Scan-N-Plan" application.  It helps gain familiarity with ROS1, to simplify later exercises.  But does not contribute anything directly to the Scan-N-Plan Application.

## Scan-N-Plan Application: Guidance

### Create ROS1 Workspace
1. ROS1 nodes and workspaces exist separately from ROS2.  Open a NEW terminal and create a workspace to store ROS1 packages:

   ```
   mkdir -p ~/ros1_ws/src
   cd ~/ros1_ws
   source /opt/ros/noetic/setup.bash
   catkin init
   catkin build
   source ~/ros1_ws/devel/setup.bash
   ```
   
   * _We have sourced the ROS1 "noetic" distribution in this terminal.  Remember to not use this terminal for ROS2 exercises!_
   * _Note the use of ROS1's build tool "catkin" to initialize and build the (empty) workspace._
   * The ROS1 build process creates a "devel" directory to hold build results during active system development.  It does support an "install" target, but this is secondary during ROS1 development.  In contrast to ROS2, which drops the "devel" directory and uses "install" always.

### Run ROS1 nodes
1. Try to run an example ROS1 node:

   ```
   rosrun roscpp_tutorials babbler
   ```
   
   * You can use tab-completion in ROS1 just like you do in ROS2.
   * You will get an error message "Failed to contact master" when trying to run the ROS1 node.  Every ROS1 system needs a running master node.  Use Ctrl-C to cancel the running node.
   
1. Start a master node using `roscore`.  Then open a new terminal window, source the ROS1 environment, and re-run the test node.

   ```
   roscore
   
   <in a new terminal window>
   source ~/ros1_ws/devel/setup.bash
   rosrun roscpp_tutorials babbler
   ```

1. Open a 3rd terminal window and source the ROS1 environment.  Inspect the running environment:

   ```
   rostopic list
   rostopic echo /babble
   rosnode list
   rosnode info /babbler
   ```
   
1. Close the babbler node and launch another demo node that has parameter support.  Experiment with setting parameter values to the ROS1 parameter server.

   ```
   rosrun turtlesim turtlesim_node
   
   <in a separate terminal window>
   rosparam list
   rosparam set /turtlesim/background_g 255
   ```
 
   * Parameter changes don't seem to do anything.  Many nodes only read the parameter values at startup.  Close and restart the turtlesim node.  The parameter values are retained on the parameter server and now change turtlesim's window color.
   * Try setting a new parameter value to an arbitrary value (`rosparam set /debug true`).  This is allowed.  The parameter server allows anyone to write/read values to the global parameter shared space.
 
 1. Close the running turtlesim node (Ctrl-C) and the roscore.  Launch a ROS1 node using a launch file:
 
    ```
    roslaunch roscpp_tutorials talker_listener.launch
    
    <in a 2nd ROS1 terminal>
    rosrun rqt_graph rqt_graph
    ```
    
    * As a convenience, roslaunch will automatically start the roscore process if needed.  This roscore is shut down when the last launch-file exits.
    
1. Kill the running launch file (Ctrl-C).

### ROS1/2 Bridge Operation
> In this portion, we will connect a publisher running in ROS1 to a subscriber running in ROS2 through the ROS bridge node.

1. In a ROS1 terminal, start `roscore` to create a ROS1 master.

1. In a BRAND NEW terminal start the ROS bridge.  It requires a very specific environment configuration, which is easiest to control when starting from a "clean" terminal.

   ```
   source /opt/ros/noetic/setup.bash
   source ~/ros2_ws/install/setup.bash
   export ROS_MASTER_URI=http://localhost:11311
   ros2 run ros1_bridge dynamic_bridge
   ```

1. In a ROS1 terminal, run the publisher node:

   ```
   rosrun roscpp_tutorials babbler
   ```
   
1. In a ROS2 terminal, run the listener node:

   ```
   ros2 run demo_nodes_cpp listener --ros-args -r chatter:=babble
   ```
 
   * You should see a stream of "I heard" messages from the listener, confirming that messages are being received from the ROS1 publisher.
   * Note the use of the "remapping" command-line args to allow the ROS2 listener (expecting a "chatter" topic) to connect to the ROS1 publisher (broadcasting a "babble" topic).
   * Review the messages in the ROS Bridge terminal to see what's printed when the bridge establishes a connection.  Kill one of the pub/sub nodes and restart it, and see what bridge messages match those events.

1. Kill all running nodes and close any open ROS1 terminals, to avoid causing confusion with future exercises.

## Challenge Exercises
* Use a ROS2 publisher and a ROS1 subscriber (opposite direction).
* Use [command-line args](http://wiki.ros.org/Remapping%20Arguments) to remap the ROS1 topic instead of the ROS2 topic.
* Create a pair of launch files to automatically restart the ROS1 and ROS2 nodes.  There is no good way to automatically start both ROS1 and ROS2 nodes from a single unified launch file.

