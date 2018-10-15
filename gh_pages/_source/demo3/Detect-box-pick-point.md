# Detect Box Pick Point
The first step in a pick and place operation is that the pick location must be found. To do that, we will leverage a 3D camera sensor and the Point Cloud Library (PCL).

The coordinate frame of the box's pick can be requested from a ros service that detects it by processing the sensor data. In this exercise, we will learn to write a ROS service and apply the neccessary filters to locate the pick location.

## Overview of the process

The perception node is launched. This registers a new service with ```roscore``` and pulls necessary parameters from the ROS parameter server. It then waits until the service is called. When the ROS perception service is called:

1) Service pulls latest point cloud from the 3D sensor
2) Point cloud is cropped to exclude areas outside of the work cell
3) RANSAC plane segmentation is used to remove the work table from the point cloud
4) Euclidean clster extraction is used to cluster the remaining points 
5) Outliers are removed
6) The largest cluster is taken as the pick object (additional logic could be added here to support multiple pick objects in view)
7) RANSAC plane segmentations is used to find the top of the box
8) The centroid of these points is calcuated
9) The service returns this pose

## Explore processing_node.launch
Open processing_node.launch in demo3_perception/launch. This file launches the perception node. Note the standalone flag which can be set to true for testing the perception without the rest of the system.

Additionally, it defines adds some associated rosparams to the parameter server. Explore these parameters. While the given values should work in simulation, it is likely that some of the PCL filter parameters will need to be changed when moving to real hardware. Here are some of them associated with the ROS setup.

* cloud_debug: true means intermediate point clouds will be published for debugging
* cloud_topic: Topic from which the service pulls the point cloud
* world_frame: Frame into which the point cloud is placed
* camera_frame: Frame associated with the camera location


## Define ROS Service
We first review the ROS service definition. This file will define the service inputs and outputs. 

* In the pick_and_place_perception/srv directory, open GetTargetPose.srv
* Ensure that the following code is in this file. Note that the service request is empty. It returns a boolean flag, a ROS message of the geometry_msgs/Pose type for the location of the center of the top of the box, and two geometry_msgs/Pose messages that define a bounding box around the top of the box.


```
# Request - empty
---
# Response - service returns a bool, a geometry_msgs/Pose, and 2 geometry_msgs/Points
bool succeeded
geometry_msgs/Pose target_pose
geometry_msgs/Point min_pt
geometry_msgs/Point max_pt

``` 
## Explore perception_node architecture

In order for a RIS service to be registered on roscore, it must be advertised similarly to a node. This is done with ```nh.advertiseService(args)```. While this is often done the main(), for our use case this is done in the constructor of the PerceptionPipeline class. The service function is then a member of that class. Creating a pipeline class has several advantages.

* It allows us to create ROS publishers as class members that are also maintained between service calls. We are using this for debugging in our case by publishing the intermediate point clouds.
* It allows us to read parameters and store them in class members that are maintained between service calls, reducing processing time.
* It allows us to instatiate multiple instances of the same pipeline.

## Complete Code

  * A template had been provided with some of the setup complete
  * Find every line that begins with the comment "''Fill Code: ''" and read the description.  Then, replace every instance of the comment  "''ENTER CODE HERE''"
 with the appropriate line of code

```
/* Fill Code:
     .
     .
     .
*/
//ENTER CODE HERE: Brief Description
```

  * The '''find_pick_client''' object in your programs can use the '''call()''' method to send a request to a ros service.

  * The ros service that receives the call will process the sensor data and return the pose for the box pick in the service structure member '''srv.response.target_pose'''.


## Build Code and Run

  * Compile the pick and place node in QT
```
Build -> Build Project
```

  * Alternatively, in a terminal cd into your workspace directory and do the following
```
catkin build
```

  * Run your node with the launch file used before which calls the processing_node.launch. While you could call that launch file directly, the environment would not be loaded, and there would be no point cloud to process.
```
roslaunch pick_and_place pick_and_place.launch
```
  * test_bed_core_node will call your service automatically. However, you can also test your code from the command line by calling ```rosservice call /find_pick``` when a point cloud is is being published on the cloud_topic (/cloud by default).

  * Some errors and warnings will be present when the template code is run. As the correct code is filled, these will disappear until a fully defined Pose message is returned from the service.

## Additional exploration (optional)

You may notice that this service does not run terribly quickly. Aside from publishing the clouds for debugging, intermediate copies of the point cloud are made that simplify the demonstration but hurt performance. Additionally, PCL has a wide variety of filters available and the solution given here is just one of many possible. Use the included timing functions to explore changes that can speed processing.


## API References


[call()](http://docs.ros.org/hydro/api/roscpp/html/classros_1_1ServiceClient.html#a8a0c9be49046998a830df625babd396f)
