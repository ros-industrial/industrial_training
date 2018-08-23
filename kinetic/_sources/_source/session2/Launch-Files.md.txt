# Launch Files
>In this exercise, we will explore starting groups of nodes at once with launch files.

## Motivation
The ROS architecture encourages engineers to use ''nodes'' as a fundamental unit of organization in their systems, and applications can quickly grow to require many nodes to operate. Opening a new terminal and running each node individually quickly becomes unfeasible. It'd be nice to have a tool to bring up groups of nodes at once. ROS ''launch'' files are one such tool. It even handles bringing ```roscore``` up and down for you.

## Reference Example

[Roslaunch Examples](http://wiki.ros.org/roslaunch/XML#Example_.launch_XML_Config_Files)

## Further Information and Resources

[Roslaunch XML Specification](http://wiki.ros.org/roslaunch/XML)

[Debugging and Launch Files](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB)

## Scan-N-Plan Application: Problem Statement
In this exercise, you will:
1. Create a new package, `myworkcell_support`.
1. Create a directory in this package called `launch`.
1. Create a file inside this directory called `workcell.launch` that:
   1. Launches `fake_ar_publisher`
   1. Launches `vision_node`

You may also choose to launch `myworkcell_core` node with the others or keep it separate.  We often configure systems with two main launch files.  In this example, `fake_ar_publisher` and `vision_node` are "environment nodes", while `myworkcell_node` is an "application" node.

1. "Environment" Launch File - driver/planning nodes, config data, etc.
1. "Application" Launch File - executes a sequence of actions for a particular application.

## Scan-N-Plan Application: Guidance

1. In your workspace, create the new package `myworkcell_support` with a dependency on `myworkcell_core`.  Rebuild and source the workspace so that ROS can find the new package:

   ``` bash
   cd ~/catkin_ws/src
   catkin create pkg myworkcell_support --catkin-deps myworkcell_core
   catkin build
   source ~/catkin_ws/devel/setup.bash
   ```

2. Create a directory for launch files (inside the new `myworkcell_support` package):

   ``` bash
   roscd myworkcell_support
   mkdir launch
   ```

3. Create a new file, `workcell.launch` (inside the `launch` directory) with the following XML skeleton:

   ``` xml
   <launch>

   </launch>
   ```

4. Insert lines to bring up the nodes outlined in the problem statement. See the reference documentation for more information:

   ``` xml
   <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" />
   <node name="vision_node" pkg="myworkcell_core" type="vision_node" />
   ```
   * _Remember: All launch-file content must be **between** the `<launch> ... </launch>` tag pair._
 
5. Test the launch file:

   ``` bash
   roslaunch myworkcell_support workcell.launch
   ```

   _Note: roscore and both nodes were automatically started.  Press _Ctrl+C_ to close all nodes started by the launch file. If no nodes are left running, roscore is also stopped._

6. Notice that none of the usual messages were printed to the console window.  Launch files will suppress console output below the **ERROR** severity level by default. To restore normal text output, add the `output="screen"` attribute to each of the nodes in your launch files:

   ``` xml
   <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" output="screen"/>
   <node name="vision_node" pkg="myworkcell_core" type="vision_node" output="screen" />

   ```
