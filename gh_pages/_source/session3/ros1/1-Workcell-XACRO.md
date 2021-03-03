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
 1. Rename the `workcell.urdf` file from the previous exercise to `workcell.xacro`

 1. Bring in the `ur_description` package into your ROS environment. You have a few options:

    1. You can install the debian packages.

       ```
       sudo apt install ros-melodic-ur-description ros-melodic-ur-kinematics
       ```

    >Note: these may or may not exist for the current distribution of ROS. If this is the case, you will need to download them from source (see next step)

    1. You can clone it from [GitHub](https://github.com/ros-industrial/universal_robot) to your catkin workspace:

       ```
       cd ~/catkin_ws/src
       git clone https://github.com/ros-industrial/universal_robot.git
       catkin build
       source ~/catkin_ws/devel/setup.bash
       ```

    >It’s not uncommon for description packages to put each “module”, “part”, or “assembly” into its own file. In many cases, a package will also define extra files that define a complete cell with the given part so that we can easily visually inspect the result. The UR package defines such a file for the UR5 ([ur5_robot.urdf.xacro](https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_description/urdf/ur5_robot.urdf.xacro)): It’s a great example for this module.

 1. Locate the xacro file that implements the UR5 macro and include it in your newly renamed `workcell.xacro` file.  Add this include line near the top of your `workcell.xacro` file, beneath the `<robot>` tag:

    ``` xml
    <xacro:include filename="$(find ur_description)/urdf/ur5.urdf.xacro" />
    ```

    >If you explore the UR5 definition file, or just about any other file that defines a Xacro macro, you’ll find a lot of uses of `${prefix}` in element names. Xacro evaluates anything inside a “${}” at run-time. It can do basic math, and it can look up variables that come to it via properties (ala-global variables) or macro parameters. Most macros will take a “prefix” parameter to allow a user to create multiple instances of said macro. It’s the mechanism by which we can make the eventual URDF element names unique, otherwise we’d get duplicate link names and URDF would complain.

 1. Including the `ur5.urdf.xacro` file does not actually create a UR5 robot in our URDF model.  It defines a macro, but we still need to call the macro to create the robot links and joints.  _Note the use of the `prefix` tag, as discussed above._

    ``` xml
    <xacro:ur5_robot prefix="" joint_limited="true"/>
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

 1. Create a new `urdf.launch` file (in the `myworkcell_support` package) to load the URDF model and (optionally) display it in rviz:

    ``` xml
    <launch>
      <arg name="gui" default="true"/>
      <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find myworkcell_support)/urdf/workcell.xacro'" />
      <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
      <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
        <param name="use_gui" value="$(arg gui)"/>
      </node>
      <node name="rviz" pkg="rviz" type="rviz" if="$(arg gui)"/>
    </launch>
    ```

 1. Check the updated URDF in RViz, using the launch file you just created:

    `roslaunch myworkcell_support urdf.launch`

    * Set the 'Fixed Frame' to 'world' and add the RobotModel and TF displays to the tree view on the left, to show the robot and some transforms.
    * Try moving the joint sliders to see the UR5 robot move.
