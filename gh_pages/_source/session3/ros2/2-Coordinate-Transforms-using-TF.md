# Coordinate Tranforms using TF
>In this exercise, we will explore the terminal and C++ commands used with TF, the transform library.

## Motivation
It’s hard to imagine a useful, physical “robot” that doesn’t move itself or watch something else move. A useful application in ROS will inevitably have some component that needs to monitor the position of a part, robot link, or tool. In ROS, the “eco-system” and library that facilitates this is called TF.
TF is a fundamental tool that allows for the lookup the transformation between any connected frames, even back through time. It allows you to ask questions like: “What was the transform between A and B 10 seconds ago.” That’s useful stuff.


## Reference Example
[ROS TF2 Listener Tutorial](https://docs.ros.org/en/humble/Tutorials/Tf2/Writing-A-Tf2-Listener-Cpp.html)

## Further Information and Resources
 * [Wiki Documentation](http://wiki.ros.org/tf2) - ROS1 version
 * [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Tf2/Tf2-Main.html)
 * [TF2 Buffer API](http://docs.ros2.org/foxy/api/tf2_ros/classtf2__ros_1_1Buffer.html) - Foxy distro

## Scan-N-Plan Application: Problem Statement
The part pose information returned by our (simulated) camera is given in the optical reference frame of the camera itself. For the robot to do something with this data, we need to transform the data into the robot’s reference frame.

Specifically, edit the service callback inside the vision_node to transform the last known part pose from `camera_frame` to the service call’s `base_frame` request field.


## Scan-N-Plan Application: Guidance

 1. Specify `tf2_ros` and `tf2_geometry_msgs` as dependencies of your core package.

    * Edit `package.xml` (+2 lines) and `CMakeLists.txt` as in previous exercises (`find_package` and `ament_target_dependencies` for the `vision_node`).

 1. Add `tf2_ros::Buffer` and `tf2_ros::TransformListener` objects to the vision node (as class members variables). 

    ``` c++
    #include <tf2_ros/buffer.h>
    #include <tf2_ros/transform_listener.h>
    ...
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    ```

 1. Also add include lines to be able to work with `geometry_msgs` types:

    ``` c++
    #include <geometry_msgs/msg/pose_stamped.hpp>
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
    ```

 1. The transform listener must be constructed using the buffer. Initialize it in the class constructor:

    ``` c++
    class Localizer : public rclcpp::Node
    {
    public:
      Localizer() : Node("vision_node"), buffer_(this->get_clock()), listener_(buffer_)
    ```

 1. Add code to the existing `localizePart` method to convert the reported target pose from its reference frame ("camera_frame") to the service-request frame:

    1. Remove the placeholder line from a previous exercise that sets the resulting pose equal to the pose from the last message

       ``` diff
       - res.pose = p->pose.pose;
       ```

    1. To perform a coordinate transformation, we will use a _stamped pose_ object which bundles a 3D pose with metadata about what coordinate system the pose is in, which is known as a _header_. These pieces of data both come from the message received by the marker publisher:

       ``` c++
       geometry_msgs::msg::PoseStamped target_pose_from_cam;
       target_pose_from_cam.header = p->header;
       target_pose_from_cam.pose = p->pose.pose;
       ```

    1. Use the buffer object to transform this `PoseStamped` object to another coordinate frame, specified by the frame in the service request:

       ``` c++
       geometry_msgs::msg::PoseStamped target_pose_from_req = buffer_.transform(
           target_pose_from_cam, req->base_frame);
       ```

       - Note: The buffer looks up the transformation between the camera frame and the base frame at the specific time when the message was first generated, which is also recorded in the header of the message.
       - There are many other _Stamped_ versions messages besides `PoseStamped`. Most of them can also be transformed to different coordinate system using the same method.

    1. Return the transformed pose in the service response. 

       ``` c++
       res->pose = target_pose_from_req.pose;
       res->success = true;
       ```

 1. Run the nodes to test the transforms:

    ```
    colcon build
    
    <in separate terminals>
    ros2 launch myworkcell_support urdf.launch.py
    ros2 launch myworkcell_support workcell.launch.py
    ```

 1. Change the "base_frame" parameter in `workcell.launch.py` (e.g. to "table"), relaunch the file, and note the different pose result.  Change the "base_frame" parameter back to "world" when you're done.

## Challenge Exercises
* Try creating a second service to return the transformation from the camera to the table. 