# Workcell XACRO
>In this exercise, we will create an XACRO file representing a simple robot workcell. This will demonstrate both URDF and XACRO elements.


## Motivation
Writing URDFs that involve more than just a few elements can quickly become a pain. Your file gets huge and duplicate items in your workspace means copy-pasting a bunch of links and joints while having to change their names just slightly. It’s really easy to make a mistake that may (or may not) be caught at startup. 
It’d be nice if we could take some guidance from programming languages themselves: define a component once, then re-use it anywhere without excessive duplication. Functions and classes do that for programs, and XACRO macros do that for URDFs. XACRO has other cool features too, like a file include system (think #include), constant variables, math expression evaluation (e.g., say PI/2.0 instead of 1.57), and more. 

## Reference Example

[Cleaning Up URDF with XACRO Tutorial](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File)

## Further Information and Resources

[Xacro Extension Documentation](http://wiki.ros.org/xacro)

[Creating a URDF for an Industrial Robot](http://wiki.ros.org/Industrial/Tutorials/Create%20a%20URDF%20for%20an%20Industrial%20Robot)

## Scan-N-Plan Application: Problem Statement
In the previous exercise we created a workcell consisting of only static geometry. In this exercise, we'll add a UR5 robot _assembly_ using XACRO tools.

Specifically, you will need to:
 1. Convert the `*.urdf` file you created in the previous sample into a XACRO file with the `xacro` extension.
 1. Include a file containing the xacro-macro definition of a `UR5`
 1. Instantiate a `UR5` in your workspace and connect it to the _table_ link.

## Scan-N-Plan Application: Guidance
 1. Rename the `workcell.urdf` file from the previous exercise to `workcell.urdf.xacro`

 1. Bring in the `ur_description` package into your ROS environment. For ROS2, it currently must be cloned and built from source

    1. Search the [UniversalRobots GitHub repositories](https://github.com/UniversalRobots) for the `ur_description` package, to find that it is available in the `Universal_Robots_ROS2_Driver` repository.  Clone the repository and a minimal set of build dependencies into your workspace:

       ```
       cd ~/ros2_ws/src
       git clone -b foxy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
       git clone -b master https://github.com/UniversalRobots/Universal_Robots_Client_Library.git
       sudo apt install ros-foxy-ur-msgs
       cd ~/ros2_ws
       colcon build
       ```

    > Inspect the UR-provided xacro files at `~/ros2_ws/src/Universal_Robots_ROS2_Driver/ur_description/urdf`.  In particular, compare `ur_macro.xacro` with `ur.urdf.xacro`.  This is a common design pattern: one xacro file defines a macro that can be called to generate a component of the workcell, while a different xacro file _calls_ that macro to actually create the final URDF model.

 1. Locate the xacro file that defines the "ur_robot" macro and include it in your newly renamed `workcell.urdf.xacro` file.  Add this include line near the top of your `workcell.urdf.xacro` file, beneath the `<robot>` tag:

    ``` xml
    <xacro:include filename="$(find ur_description)/urdf/ur_macro.xacro"/>
    ```

    >If you explore the ur_macro definition file, or just about any other file that defines a Xacro macro, you’ll find a lot of uses of `${prefix}` in element names. Xacro evaluates anything inside a “${}” at run-time. It can do basic math, and it can look up variables that come to it via properties (ala-global variables) or macro parameters. Most macros will take a “prefix” parameter to allow a user to create multiple instances of said macro. It’s the mechanism by which we can make the eventual URDF element names unique, otherwise we’d get duplicate link names and URDF would complain.

 1. Including the `ur_macro.xacro` file does not actually create a UR5 robot in our URDF model.  It defines a macro, but we still need to call the macro to create the robot links and joints.  _Note the use of the `prefix` tag, as discussed above.  An additional "ur_type" argument tells the macro which model of UR-robot to generate._

    ``` xml
    <!-- ur arm instantiation -->
    <xacro:arg name="ur_type" default="ur5"/>
    <xacro:arg name="use_fake_hardware" default="true"/>
    <xacro:arg name="fake_sensor_commands" default="true"/>
    <xacro:ur_robot
        prefix=""
        joint_limits_parameters_file="$(find ur_description)/config/$(arg ur_type)/joint_limits.yaml"
        kinematics_parameters_file="$(find ur_description)/config/$(arg ur_type)/default_kinematics.yaml"
        physical_parameters_file="$(find ur_description)/config/$(arg ur_type)/physical_parameters.yaml"
        visual_parameters_file="$(find ur_description)/config/$(arg ur_type)/visual_parameters.yaml"
        use_fake_hardware="$(arg use_fake_hardware)"
        fake_sensor_commands="$(arg fake_sensor_commands)"/>
    ```

    >Macros in Xacro are just fancy wrappers around copy-paste. You make a macro and it gets turned into a chunk of links and joints. You still have to connect the rest of your world to that macro’s results. This means you have to look at the macro and see what the base link is and what the end link is. Hopefully your macro follows a standard, like the ROS-Industrial one, that says that base links are named “base_link” and the last link is called “tool0”.

 1. Connect the UR5 `base_link` to your existing static geometry with a fixed link.

    ``` xml
    <joint name="table_to_robot" type="fixed">
      <parent link="table"/>
      <child link="base_link"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>
    ```

 1. Create a new `urdf.launch.py` file (in the `myworkcell_support` package) to load the URDF model and (optionally) display it in rviz. The launch file starts with several utility functions that are useful for assisting the launch process. This particular file uses `get_package_file` to get the path of the `workcell.urdf.xacro` file you've created, and `xacro.process_file` to generate the URDF as a string object. This URDF string is then passed to a `robot_state_publisher` node as a parameter just as was done manually in the previous exercise.

    ```py
    import os
    import launch
    import launch_ros
    import xacro
    
    from ament_index_python import get_package_share_directory
    
    def get_package_file(package, file_path):
        """Get the location of a file installed in an ament package"""
        package_path = get_package_share_directory(package)
        absolute_file_path = os.path.join(package_path, file_path)
        return absolute_file_path
    
    def generate_launch_description():
        xacro_file = get_package_file('myworkcell_support', 'urdf/workcell.urdf.xacro')
        urdf = xacro.process_file(xacro_file).toprettyxml(indent='  ')
    
        return launch.LaunchDescription([
            launch_ros.actions.Node(
                name='robot_state_publisher',
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': urdf}],
            ),
            launch_ros.actions.Node(
                name='joint_state_publisher_gui',
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                output='screen',
            ),
            launch_ros.actions.Node(
                name='rviz',
                package='rviz2',
                executable='rviz2',
                output='screen',
            )
        ])
    ```

 1. Rebuild your workspace and check the updated URDF in RViz, using the launch file you just created:

    `ros2 launch myworkcell_support urdf.launch.py`

    * If you didn't save your RViz config last time, you may need to repeat the configuration steps: set the 'Fixed Frame' to 'world' and add the RobotModel and TF displays to the tree view on the left.
    * Try moving the joint sliders in the separate GUI window that appears to see the UR5 robot move.

## Challenge Exercise
* Try adding another instance of the robot model in the scene in a different position and orienatation. 
