# Launch Files
>In this exercise, we will explore starting groups of nodes at once with launch files.

## Motivation
The ROS architecture encourages engineers to use ''nodes'' as a fundamental unit of organization in their systems, and applications can quickly grow to require many nodes to operate. Opening a new terminal and running each node individually quickly becomes unfeasible. It'd be nice to have a tool to bring up groups of nodes at once. ROS ''launch'' files are one such tool.

## Reference Example

[Launching a ROS System](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html)

## Further Information and Resources

[ROS2 python launch Node action](https://github.com/ros2/launch_ros/blob/humble/launch_ros/launch_ros/actions/node.py#L113-L200)

## Scan-N-Plan Application: Problem Statement
In this exercise, you will:
1. Create a new package, `myworkcell_support`.
1. Create a directory in this package called `launch`.
1. Create a file inside this directory called `workcell.launch.py` that:
   1. Launches `fake_ar_publisher`
   1. Launches `vision_node`

You may also choose to launch `myworkcell_core` node with the others or keep it separate.  We often configure systems with two main launch files.  In this example, `fake_ar_publisher` and `vision_node` are "environment nodes", while `myworkcell_node` is an "application" node.

1. "Environment" Launch File - driver/planning nodes, config data, etc.
1. "Application" Launch File - executes a sequence of actions for a particular application.

## Scan-N-Plan Application: Guidance

1. In your workspace, create the new package `myworkcell_support` with a dependency on `myworkcell_core`.  Rebuild and source the workspace so that ROS can find the new package:

   ``` bash
   cd ~/ros2_ws/src
   ros2 pkg create myworkcell_support --dependencies myworkcell_core
   ```

   Then run `colcon build` in your build terminal and source your running terminal.

   ```
   source ~/ros2_ws/install/setup.bash
   ```

1. Create a directory for launch files (inside the new `myworkcell_support` package):

   ``` bash
   cd src/myworkcell_support
   mkdir launch
   ```

1. Create a new file, `workcell.launch.py` (inside the `launch` directory) with the following skeleton:

   ``` py
   import launch
   import launch_ros

   def generate_launch_description():
       return launch.LaunchDescription([
           # launch actions here...
       ])
   ```
1. In the space marked by the comment about 'launch actions', insert lines to bring up the nodes outlined in the problem statement. See the reference documentation for more information:

   ``` py
   launch_ros.actions.Node(
       name='fake_ar_publisher_node',
       package='fake_ar_publisher',
       executable='fake_ar_publisher_node',
   ),
   launch_ros.actions.Node(
       name='vision_node',
       package='myworkcell_core',
       executable='vision_node',
   ),
   ```

   * There are other options you can set for these `Node` actions, but `name`, `package`, and `executable` are the most common ones.
 
5. Try to run the launch file:

   ``` bash
   ros2 launch myworkcell_support workcell.launch.py
   ```

   You should see an error that the launch file was not found. This is because it only exists in the `src/` directory, and the `ros2` tool will only work with files in the `install/` directory. Therefore, the launch file must be _installed_.

1. Add an installation rule to `CMakeLists.txt` in your `myworkcell_support` package, after the `find_package` section, and before the `BUILD_TESTING` section:

   ``` cmake
   install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
   ```

1. Now build the workspace to install the launch file and try to run it again:

   ``` bash
   ros2 launch myworkcell_support workcell.launch.py
   ```

   _Note: Both nodes were automatically started. Press _Ctrl+C_ to close all nodes started by the launch file._

1. The expected behavior is that you should see none of the usual messages printed to the console window.  Launch files will suppress console output below the **ERROR** severity level by default. To restore normal text output, add the `output='screen'` argument to each of the nodes in your launch file (see below).  _Current versions of ROS do not seem to follow this behavior, though, and output INFO/WARN messages to screen regardless of `output` setting._

   ``` py
   launch_ros.actions.Node(
       name='fake_ar_publisher_node',
       package='fake_ar_publisher',
       executable='fake_ar_publisher_node',
       output='screen',
   ),
   launch_ros.actions.Node(
       name='vision_node',
       package='myworkcell_core',
       executable='vision_node',
       output='screen',
   ),
   ```

   * Even without the `output='screen'` argument, the logging statements are still broadcast to any ROS nodes that listen to the global `/rosout` topic and captured in a log file.
   * _Important_: Remember that `ros2` works on the launch file in the `install/` directory and so you won't see the new behavior without running `colcon build` again to reinstall the file. To avoid having to do this for every change during development, you can run `colcon build` with a `--symlink-install` option which will install a link to the file in the `src/` directory so any changes will be seen immediately.
   
## Challenge Exercise
* ROS2 also supports [XML] and YAML launch files.  Review the example [here](https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html) and try to recreate the launch file we just made in XML or YAML instead.
