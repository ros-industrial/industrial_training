# Coordinate Tranforms using TF
>In this exercise, we will explore the terminal and C++ commands used with TF, the transform library.

## Motivation
It’s hard to imagine a useful, physical “robot” that doesn’t move itself or watch something else move. A useful application in ROS will inevitably have some component that needs to monitor the position of a part, robot link, or tool. In ROS, the “eco-system” and library that facilitates this is called TF.
TF is a fundamental tool that allows for the lookup the transformation between any connected frames, even back through time. It allows you to ask questions like: “What was the transform between A and B 10 seconds ago.” That’s useful stuff.


## Reference Example
[ROS TF2 Listener Tutorial](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29)

## Further Information and Resources
 * [Wiki Documentation](http://wiki.ros.org/tf2)
 * [TF2 Tutorials](http://wiki.ros.org/tf2/Tutorials)
 * [TF2 Buffer API](http://docs.ros.org/melodic/api/tf2_ros/html/c++/classtf2__ros_1_1Buffer.html)

## Scan-N-Plan Application: Problem Statement
The part pose information returned by our (simulated) camera is given in the optical reference frame of the camera itself. For the robot to do something with this data, we need to transform the data into the robot’s reference frame.

Specifically, edit the service callback inside the vision_node to transform the last known part pose from `camera_frame` to the service call’s `base_frame` request field.


## Scan-N-Plan Application: Guidance

 1. Specify `tf2_ros` and `tf2_geometry_msgs` as dependencies of your core package.

    * Edit `package.xml` (+2 lines) and `CMakeLists.txt` (+4 lines) as in previous exercises

 1. Add `tf2_ros::Buffer` and `tf2_ros::TransformListener` objects to the vision node (as class members variables). 

    ``` c++
    #include <tf2_ros/buffer.h>
    #include <tf2_ros/transform_listener.h>
    #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
    ...
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    ```

 1. The transform listener must be constructed using the buffer. Initialize it in the class constructor:

    ``` c++
    class Localizer
    {
    public:
      Localizer(ros::NodeHandle& nh) : listener_(buffer_)
    ```

 1. Add code to the existing `localizePart` method to convert the reported target pose from its reference frame ("camera_frame") to the service-request frame:

    1. Remove the placeholder line from a previous exercise that sets the resulting pose equal to the pose from the last message

       ``` diff
       - res.pose = p->pose.pose;
       ```

    1. To perform a coordinate transformation, we will use a _stamped pose_ object which bundles a 3D pose with metadata about what coordinate system the pose is in, which is known as a _header_. These pieces of data both come from the message received by the marker publisher:

       ``` c++
       geometry_msgs::PoseStamped target_pose_from_cam;
       target_pose_from_cam.header = p->header;
       target_pose_from_cam.pose = p->pose.pose;
       ```

    1. Use the buffer object to transform this `PoseStamped` object to another coordinate frame, specified by the frame in the service request:

       ``` c++
       geometry_msgs::PoseStamped target_pose_from_req = buffer_.transform(
           target_pose_from_cam, req.base_frame);
       ```
      - Note: The buffer looks up the transformation between the camera frame and the base frame at the specific time when the message was first generated, which is also recorded in the header of the message.
      - There are many other _Stamped_ versions messages besides `PoseStamped`. Most of them can also be transformed to different coordinate system using the same method.

    1. Return the transformed pose in the service response. 

       ``` c++
       res.pose = target_pose_from_req.pose;
       ```

 1. Run the nodes to test the transforms:

    ```
    catkin build
    roslaunch myworkcell_support urdf.launch
    roslaunch myworkcell_support workcell.launch
    ```

 1. Change the "base_frame" parameter in `workcell.launch` (e.g. to "table"), relaunch the `workcell.launch` file, and note the different pose result.  Change the "base_frame" parameter back to "world" when you're done.
