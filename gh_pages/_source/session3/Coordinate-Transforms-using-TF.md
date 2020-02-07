# Coordinate Tranforms using TF
>In this exercise, we will explore the terminal and C++ commands used with TF, the transform library.

## Motivation
It’s hard to imagine a useful, physical “robot” that doesn’t move itself or watch something else move. A useful application in ROS will inevitably have some component that needs to monitor the position of a part, robot link, or tool. In ROS, the “eco-system” and library that facilitates this is called TF.
TF is a fundamental tool that allows for the lookup the transformation between any connected frames, even back through time. It allows you to ask questions like: “What was the transform between A and B 10 seconds ago.” That’s useful stuff.


## Reference Example
[ROS TF Listener Tutorial](http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20(C%2B%2B))

## Further Information and Resources
 * [Wiki Documentation](http://wiki.ros.org/tf)
 * [TF Tutorials](http://wiki.ros.org/tf/Tutorials)
 * [TF Listener API](http://docs.ros.org/melodic/api/tf/html/)

## Scan-N-Plan Application: Problem Statement
The part pose information returned by our (simulated) camera is given in the optical reference frame of the camera itself. For the robot to do something with this data, we need to transform the data into the robot’s reference frame.

Specifically, edit the service callback inside the vision_node to transform the last known part pose from `camera_frame` to the service call’s `base_frame` request field.


## Scan-N-Plan Application: Guidance

 1. Specify `tf` as a dependency of your core package.

    * Edit `package.xml` (1 line) and `CMakeLists.txt` (2 lines) as in previous exercises

 1. Add a `tf::TransformListener` object to the vision node (as a class member variable). 

    ``` c++
    #include <tf/transform_listener.h>
    ...
    tf::TransformListener listener_;
    ```

 1. Add code to the existing `localizePart` method to convert the reported target pose from its reference frame ("camera_frame") to the service-request frame:

    1. Remove the placeholder line from a previous exercise that sets the resulting pose equal to the pose from the last message

       ``` diff
       - res.pose = p->pose.pose;
       ```

    1. For better or worse, ROS uses lots of different math libraries. You’ll need to transform the over-the-wire format of `geometry_msgs::Pose` into a `tf::Transform object`:

       ``` c++
       tf::Transform cam_to_target;
       tf::poseMsgToTF(p->pose.pose, cam_to_target);
       ```

    1. Use the listener object to lookup the latest transform between the `request.base_frame` and the reference frame from the `ARMarker` message (which should be "camera_frame"):

       ``` c++
       tf::StampedTransform req_to_cam;
       listener_.lookupTransform(req.base_frame, p->header.frame_id, ros::Time(0), req_to_cam);
       ```

    1. Using the above information, transform the object pose into the target frame.

       ``` c++
       tf::Transform req_to_target;
       req_to_target = req_to_cam * cam_to_target;
       ```

    1. Return the transformed pose in the service response. 

       ``` c++
       tf::poseTFToMsg(req_to_target, res.pose);
       ```

 1. Run the nodes to test the transforms:

    ```
    catkin build
    roslaunch myworkcell_support urdf.launch
    roslaunch myworkcell_support workcell.launch
    ```

 1. Change the "base_frame" parameter in `workcell.launch` (e.g. to "table"), relaunch the `workcell.launch` file, and note the different pose result.  Change the "base_frame" parameter back to "world" when you're done.
 
 1. This tutorial is meant to show a very basic use case of TF.  However, it is recommended to use `tf2`, which has a similar interface but is more robust. More information on `tf2` can be found [here](http://wiki.ros.org/tf2)
