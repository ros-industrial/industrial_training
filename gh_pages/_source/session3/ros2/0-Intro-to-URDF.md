# Introduction to URDF
>In this exercise, we will explore how to describe a robot in the URDF format.


## Motivation
Many of the coolest and most useful capabilities of ROS and its community involve things like collision checking and dynamic path planning. It’s frequently useful to have a code-independent, human-readable way to describe the geometry of robots and their cells. Think of it like a textual CAD description: “part-one is 1 meter left of part-two and has the following triangle-mesh for display purposes.”
The Unified Robot Description Format (URDF) is the most popular of these formats today. This module will walk you through creating a simple robot cell that we’ll expand upon and use for practical purposes later.

## Reference Example

[Building a Visual Robot Model with URDF from Scratch](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch)

## Further Information and Resources

* [XML Specification](http://wiki.ros.org/urdf/XML)
* [ROS Tutorials](http://wiki.ros.org/urdf/Tutorials)
* [XACRO Extensions](http://wiki.ros.org/xacro)
* [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter)

## Scan-N-Plan Application: Problem Statement
We have the software skeleton of our Scan-N-Plan application, so let’s take the next step and add some physical context. The geometry we describe in this exercise will be used to:
1. Perform collision checking
1. Understand robot kinematics
1. Perform transformation math
Your goal is to describe a workcell that features:
1. An origin frame called `world`
1. A separate frame with “table” geometry (a flat rectangular prism)
1. A frame (geometry optional) called `camera_frame` that is oriented such that its Z axis is flipped relative to the Z axis of `world`

## Scan-N-Plan Application: Guidance
*Note: If you have not completed the previous tutorials, copy myworkcell_core and myworkcell_support packages from ~/industrial-training/exercises/2.3/src and git clone https://github.com/jmeyer1292/fake_ar_publisher.git into your current workspace src folder. 

1. It’s customary to put describing files that aren’t code into their own “support” package. URDFs typically go into their own subfolder ''urdf/''. See the [abb_irb2400_support](https://github.com/ros-industrial/abb/tree/kinetic-devel/abb_irb2400_support) package. Add a `urdf` sub-folder to your application support package.
1. Create a new `workcell.urdf` file inside the `myworkcell_support/urdf/` folder and insert the following XML skeleton:

   ``` xml
   <?xml version="1.0" ?>
   <robot name="myworkcell" xmlns:xacro="http://ros.org/wiki/xacro">
   </robot>
   ```

1. Add the required links. See the [irb2400_macro.xacro](https://github.com/ros-industrial/abb/blob/84825661073a18e33b68bb01b5bf371edd2efd49/abb_irb2400_support/urdf/irb2400_macro.xacro#L54-L69) example from an ABB2400.  Remember that all URDF tags must be placed **between** the `<robot> ... </robot>` tags.

   1. Add the `world` frame as a "virtual link" (no geometry).

      ```
      <link name="world"/>
      ```

   1. Add the `table` frame, and be sure to specify both collision & visual geometry tags. See the `box` type in the XML specification.

      ``` xml
      <link name="table">
        <visual>
          <geometry>
            <box size="1.0 1.0 0.05"/>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <box size="1.0 1.0 0.05"/>
          </geometry>
        </collision>
      </link>
      ```

   1. Add the `camera_frame` frame as another virtual link (no geometry).

      ```
      <link name="camera_frame"/>
      ```

   1. Connect your links with a pair of fixed joints  Use an `rpy` tag in the `world_to_camera` joint to set its orientation as described in the introduction.

      ``` xml
      <joint name="world_to_table" type="fixed">
        <parent link="world"/>
        <child link="table"/>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
      </joint>

      <joint name="world_to_camera" type="fixed">
        <parent link="world"/>
        <child link="camera_frame"/>
        <origin xyz="-0.25 -0.5 1.25" rpy="0 3.14159 0"/>
      </joint>
      ```

   1. It helps to visualize your URDF as you add links, to verify things look as expected:

      ```
      roslaunch urdf_tutorial display.launch model:=`rospack find myworkcell_support`/urdf/workcell.urdf
      ```

  _If nothing shows up in Rviz, you may need to change the base frame in RVIZ (left panel at top) to the name of one of the links in your model._
