Place Box
=========

  In this exercise, we will move the robot through the place motions while
  avoiding obstacles with an attached payload. In addition, the gripper must
  be opened or close at the appropriate time in order to complete the task.


Locate Function
---------------

* In the main program, locate the function call to ``application.place_box()``.
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

* At this point your exercise is complete and the robot should move through
  the pick and place motions and then back to the wait pose. Congratulations!


API References
--------------

* |execute()|_

* `MoveGroupInterface class <http://docs.ros.org/melodic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html>`_


.. |execute()| replace:: `execute()`_

.. _execute(): http://docs.ros.org/melodic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html#add236df4ab9ba7b7011ec53f8aa9c026
