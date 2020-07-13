Open Gripper
============

  In this exercise, the objective is to use a "grasp action client" to send a
  grasp goal that will open the gripper.


Locate Function
---------------

* In the main program, locate the function call to
  ``application.set_gripper()``.
* Go to the source file of that function by clicking in any part of the
  function and pressing :kbd:`F2` in QtCreator.
* Remove the first line containing the following ``ROS_ERROR_STREAM ...`` so
  that the program runs.


Complete Code
-------------

* Find every line that begins with the comment ``Fill Code:`` and read the
  description. Then, replace every instance of the comment ``ENTER CODE HERE``
  with the appropriate line of code.

  .. code-block:: cpp

    /* Fill Code:
        .
        .
        .
    */
    /* ========  ENTER CODE HERE ======== */

* The ``grasp_goal.goal`` property can take on three possible values:

  .. code-block:: cpp

    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP;
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE;
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP;

* Once the grasp flag has been set you can send the goal through the grasp
  action client


Build Code and Run
------------------

* Compile the pick and place node:

  * In QTCreator: :menuselection:`Build --> Build Project`

  * Alternatively, in a terminal:

    .. code-block:: shell

      catkin build collision_avoidance_pick_and_place

* Run the supporting nodes with the launch file:

  .. code-block:: shell

    roslaunch collision_avoidance_pick_and_place ur5_setup.launch

* In another terminal, run your node with the launch file:

  .. code-block:: shell

    roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch

* If the task succeeds you will see something like the following in the
  terminal (below). The robot will not move, only gripper I/O is triggered:

  .. code-block:: text

    [ INFO] [1400553290.464877904]: Move wait Succeeded
    [ INFO] [1400553290.720864559]: Gripper opened
    [ERROR] [1400553290.720985315]: detect_box_pick is not implemented yet.  Aborting.


API References
--------------

* `sendGoal() <http://docs.ros.org/melodic/api/actionlib/html/classactionlib_1_1SimpleActionClient.html#ae6a2e6904495e7c20c59e96af0d86801>`_
