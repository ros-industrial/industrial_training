# Introduction to Descartes Path Planning
>In this exercise, we will use what was learned in the previous exercises by creating a Descartes planner to create a robot path.

## Motivation
MoveIt! is a framework meant primarily for performing "free-space" motion where the objective is to move a robot from point A to point B and you don't particularly care about how that gets done. These types of problems are only a subset of frequently performed tasks. Imagine any manufacturing ''process'' like welding or painting. You very much care about where that tool is pointing the entire time the robot is at work.

This tutorial introduces you to Descartes, a ''Cartesian'' motion planner meant for moving a robot along some process path. It's only one of a number of ways to solve this kind of problem, but it's got some neat properties:
 * It's deterministic and globally optimum (to a certain search resolution).
 * It can search redundant degrees of freedom in your problem (say you have 7 robot joints or you have a process where the tool's Z-axis rotation doesn't matter).

## Reference Example
[Descartes Tutorial](http://wiki.ros.org/descartes/Tutorials/Getting%20Started%20with%20Descartes)

## Further Information and Resources
[Descartes Wiki](http://wiki.ros.org/descartes)

APIs:
 * [descartes_core::PathPlannerBase](http://docs.ros.org/indigo/api/descartes_core/html/classdescartes__core_1_1PathPlannerBase.html)
 * [descartes_planner::DensePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1DensePlanner.html)
 * [descartes_planner::SparsePlanner](http://docs.ros.org/indigo/api/descartes_planner/html/classdescartes__planner_1_1SparsePlanner.html)

## Scan-N-Plan Application: Problem Statement
In this exercise, you will add a new node to your Scan-N-Plan application, based on a reference template, that:
 1. Takes the nominal pose of the marker as input through a ROS service.
 1. Produces a joint trajectory that commands the robot to trace the perimeter of the marker (as if it is dispensing adhesive).

## Scan-N-Plan Application: Guidance
In the interest of time, we've included a file, `descartes_node.cpp`, that:
 1. Defines a new node & accompanying class for our Cartesian path planning.
 1. Defines the actual service and initializes the Descartes library.
 1. Provides the high level work flow (see planPath function).

Left to you are the details of:
 1. Defining a series of Cartesian poses that comprise a robot “path”.
 1. Translating those paths into something Descartes can understand.

### Setup workspace
 1. Clone the Descartes repository into your workspace src/ directory.
     
    ```bash
    cd ~/catkin_ws/src
    git clone -b melodic-devel https://github.com/ros-industrial-consortium/descartes.git
    ```

 1. Copy over the `ur5_demo_descartes` package into your workspace src/ directory.

    ```bash
    cp -r ~/industrial_training/exercises/4.1/src/ur5_demo_descartes .
    ```

 1. Copy over the `descartes_node_unfinished.cpp` into your core package's src/ folder and rename it `descartes_node.cpp`.

    ```bash
    cp ~/industrial_training/exercises/4.1/src/descartes_node_unfinished.cpp myworkcell_core/src/descartes_node.cpp
    ```

 1. Add dependencies for the following packages in the `CMakeLists.txt` & `package.xml` files, as in previous exercises.
    * `ur5_demo_descartes`
    * `descartes_trajectory`
    * `descartes_planner`
    * `descartes_utilities`
    * `eigen_conversions`

 1. Create rules in the `myworkcell_core` package's `CMakeLists.txt` to build a new node called `descartes_node`.  As in previous exercises, add these lines near similar lines in the template file (not as a block as shown below).
     
    ```cmake
    add_executable(descartes_node src/descartes_node.cpp)
    add_dependencies(descartes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(descartes_node ${catkin_LIBRARIES})
    ```

### Complete Descartes Node

We will create a Service interface to execute the Descartes planning algorithm.

 1. Define a new service named `PlanCartesianPath.srv` in the `myworkcell_core` package's `srv/` directory.  This service takes the central target position and computes a joint trajectory to trace the target edges.

    ```
    # request
    geometry_msgs/Pose pose

    ---

    # response
    trajectory_msgs/JointTrajectory trajectory
    ```

 1. Add the newly-created service file to the `add_service_file()` rule in the package's `CMakeLists.txt`.
     
 1. Since our new service references a message type from another package, we'll need to add that other package (`trajectory_msgs`) as a dependency in the `myworkcell_core` `CMakeLists.txt` (3 lines) and `package.xml` (1 line) files.  

 1. Review `descartes_node.cpp` to understand the code structure.  In particular, the `planPath` method outlines the main sequence of steps.

 1. Search for the TODO commands in the Descartes node file and expand on those areas:
    1. In `makeToolPoses`, generate the remaining 3 sides of a path tracing the outside of our "AR Marker" target part.
    1. In `makeDescartesTrajectory`, convert the path you created into a Descartes Trajectory, one point at a time.
       * _Don't forget to transform each nominal point by the specified reference pose: `ref * point`_
    1. In `makeTolerancedCartesianPoint`, create a `new AxialSymmetricPt` from the given pose.
       * See [here](http://docs.ros.org/indigo/api/descartes_trajectory/html/classdescartes__trajectory_1_1AxialSymmetricPt.html) for more documentation on this point type
       * Allow the point to be symmetric about the Z-axis (`AxialSymmetricPt::Z_AXIS`), with an increment of 90 degrees (PI/2 radians)

 1. Build the project, to make sure there are no errors in the new `descartes_node`
 
### Update Workcell Node

With the Descartes node completed, we now want to invoke its logic by adding a new `ServiceClient` to the primary workcell node. The result of this service is a joint trajectory that we must then execute on the robot. This can be accomplished in many ways; here we will call the `JointTrajectoryAction` directly.

 1. In `myworkcell_node.cpp`, add include statements for the following headers:

    ```c++
    #include <actionlib/client/simple_action_client.h>
    #include <control_msgs/FollowJointTrajectoryAction.h>
    #include <myworkcell_core/PlanCartesianPath.h>
    ```

    _You do not need to add new dependenies for these libraries/messages, because they are pulled in transitively from moveit._

 1. In your ScanNPlan class, add new private member variables: a ServiceClient for the `PlanCartesianPath` service and an action client for `FollowJointTrajectoryAction`:
    ```c++
    ros::ServiceClient cartesian_client_;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
    ```

 1. Initialize these new objects in your constructor. Note that the action client has to be initialized in what is called the `initializer list`.
    ```c++
    ScanNPlan(ros::NodeHandle& nh) : ac_("joint_trajectory_action", true)
    {
      // ... code
      cartesian_client_ = nh.serviceClient<myworkcell_core::PlanCartesianPath>("plan_path");
    }
    ```
 1. At the end of the `start()` function, create a new Cartesian service and make the service request:

    ```c++
    // Plan cartesian path
    myworkcell_core::PlanCartesianPath cartesian_srv;
    cartesian_srv.request.pose = move_target;
    if (!cartesian_client_.call(cartesian_srv))
    {
      ROS_ERROR("Could not plan for path");
      return;
    }
    ```

 1. Continue adding the following lines, to execute that path by sending an action directly to the action server (bypassing MoveIt):

    ```c++
    // Execute descartes-planned path directly (bypassing MoveIt)
    ROS_INFO("Got cart path, executing");
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = cartesian_srv.response.trajectory;
    ac_.sendGoal(goal);
    ac_.waitForResult();
    ROS_INFO("Done");
    ```

 1. Build the project, to make sure there are no errors in the new `descartes_node`

### Test Full Application

 1. Create a new `setup.launch` file (in `workcell_support` package) that brings up everything except your workcell_node:

    ``` xml
    <include file="$(find myworkcell_moveit_config)/launch/myworkcell_planning_execution.launch"/>
    <node name="fake_ar_publisher" pkg="fake_ar_publisher" type="fake_ar_publisher_node" />
    <node name="vision_node" type="vision_node" pkg="myworkcell_core" output="screen"/>
    <node name="descartes_node" type="descartes_node" pkg="myworkcell_core" output="screen"/>
    ```

 1. Run the new setup file, then your main workcell node:

    ``` bash
    roslaunch myworkcell_support setup.launch
    rosrun myworkcell_core myworkcell_node
    ```

    It's difficult to see what's happening with the rviz planning-loop always running.  Disable this loop animation in rviz (Displays -> Planned Path -> Loop Animation), then rerun `myworkcell_node`.

### Hints and Help

Hints:
 * The path we define in `makeToolPoses()` is relative to some known reference point on the part you are working with. So a tool pose of (0, 0, 0) would be exactly at the reference point, and not at the origin of the world coordinate system.
 * In `makeDescartesTrajectorty(...)` we need to convert the relative tool poses into world coordinates using the “ref” pose.
 * In `makeTolerancedCartesianPoint(...)` consider the following documentation for specific implementations of common joint trajectory points:
   * <http://docs.ros.org/indigo/api/descartes_trajectory/html/>
 * For additional help, review the completed reference code at `~/industrial_training/exercises/4.1/src`
