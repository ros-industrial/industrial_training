# General Instructions
>In this exercise, we will demonstrate how to run the demo as you progress through the exercises.  Also, it will show how to run the system in simulation mode and on the real robot.

## Main Objective

 In general, you will be implementing a `plan_and_run` node incrementally.  This means that in each exercise you will be adding individual pieces that are needed to complete the full application demo. Thus, when an exercise is completed, run the demo in simulation mode in order to verify your results. Only when all of the exercises are finished should you run it on the real robot.

## Complete Exercises

 1. To complete an exercise, open the corresponding source file under the `src/plan_and_run/src/tasks/` directory. For instance, in Exercise 1 you will open `load_parameters.cpp`.

 1. Take a minute to read the header comments for specific instructions on how to complete this particular exercise. For instance, the `load_parameters.cpp`  file contains the following instructions and hints:

``` c++
/* LOAD PARAMETERS
  Goal:
    - Load missing application parameters into the node from the ros parameter server.
    - Use a private NodeHandle in order to load parameters defined in the node's namespace.
  Hints:
    - Look at how the 'config_' structure is used to save the parameters.
    - A private NodeHandle can be created by passing the "~" string to its constructor.
*/
```

 1. Do not forget to comment out the line:

 ``` c++
ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);
 ``` 

 This line is usually located at the beginning of each function.  Omitting this step will cause the program to exit immediately when it reaches this point.

 1. When you run into a comment block starting with `/*  Fill Code:` this means that the line(s) of code that follow are incorrect, commented out or incomplete at best. Read the instructions following `Fill Code` and complete the task as per described.  Below is an example of instructions comment block:

``` c++
  /*  Fill Code:
   * Goal:
   * - Create a private handle by passing the "~" string to its constructor
   * Hint:
   * - Replace the string in ph below with "~" to make it a private node.
   */
```

 1. The ```[ COMPLETE HERE ]``` entries are bound to be replaced by the appropriate `code entry`.  The right code entries could either be program variables, strings or numeric constants.  An example is shown below:

``` c++
ros::NodeHandle ph("[ COMPLETE HERE ]: ?? ");
```

In this case the correct replacement would be the string `"~"`, so this line would look like this:

``` c++
ros::NodeHandle ph("~");
```

 1. As you are completing each task in this exercise, you can run the demo (see following sections) to verify that it is completed properly.

## Run Demo in Simulation Mode

 1. In a terminal, run the setup launch file as follows:

```
roslaunch plan_and_run demo_setup.launch
```

  * When the virtual robot is ready , Rviz should be up and running with a UR5 arm in the home position and you will see the following messages in the terminal:

```
      .
      .
      .
********************************************************
* MoveGroup using: 
*     - CartesianPathService
*     - ExecutePathService
*     - KinematicsService
*     - MoveAction
*     - PickPlaceAction
*     - MotionPlanService
*     - QueryPlannersService
*     - StateValidationService
*     - GetPlanningSceneService
*     - ExecutePathService
********************************************************

[ INFO] [1430359645.694537917]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[ INFO] [1430359645.694700640]: MoveGroup context initialization complete

All is well! Everyone is happy! You can start planning now!
```

  * This launch file only needs to be run once.
 
 1. In a separate terminal, run the application launch file:

```
roslaunch plan_and_run demo_run.launch 
```

  * Look in the Rviz window and the arm should start moving.  


## Run Demo on the Real Robot

   **Notes**

   * Make sure you can `ping` the robot and that no obstacle is within its range.

 1. In a terminal, run the setup launch file as follows:

```
roslaunch plan_and_run demo_setup.launch sim:=false robot_ip:=000.000.0.00 
```

  **Notes:**

   * Enter the actual IP address of the robot into the `robot_ip` argument.  The robot model in Rviz should be in about the same position as the real robot.


 1. In a separate terminal, run the application launch file:

```
roslaunch plan_and_run demo_run.launch 
```

  * This time the real robot should start moving.
