Pick Up Box
===========

  In this exercise, we will move the robot through the pick motion while
  avoiding obstacles in the environment. This will be accomplished by
  planning for each pose and closing or opening the vacuum gripper when
  apropriate. Also, we will demonstrate how to create a motion plan that
  MoveIt! can understand and solve.


Locate Function
---------------

* In the main program, locate the function call to
  ``application.pickup_box()``.
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

* Inspect the ``set_attached_object`` method to understand how to manipulate a
  ``robot_state`` object which will then be used to construct a motion plan.
* Inspect the ``create_motion_plan`` method to see how an entire motion plan
  request is defined and sent.
* The |execute()|_ method sends a motion plan to the robot.


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

* The robot should go through the pick moves (approach, pick and retreat) in
  addition to the moves from the previous exercises. In the terminal you will
  see something like:

  .. code-block:: text

    [ INFO] [1400555978.404435764]: Execution completed: SUCCEEDED
    [ INFO] [1400555978.404919764]: Pick Move 2 Succeeded
    [ERROR] [1400555978.405061541]: create_place_moves is not implemented yet.  Aborting.


API References
--------------

* |execute()|_

* `MoveGroupInterface class <http://docs.ros.org/melodic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html>`_


.. |execute()| replace:: `execute()`_

.. _execute(): http://docs.ros.org/melodic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html#add236df4ab9ba7b7011ec53f8aa9c026
