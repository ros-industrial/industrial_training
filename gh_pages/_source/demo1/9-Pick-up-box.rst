Pick Up Box
===========

  In this exercise, we will move the robot through the pick motion while
  avoiding obstacles in the environment. This will be accomplished by
  planning for each pose and closing or opening the vacuum gripper when
  appropriate. Also, we will demonstrate how to create a motion plan that
  MoveIt! can understand and solve.


Locate Function
---------------

* In the main program, locate the function call to
  ``application.doBoxPickup()``.
* Go to the source file of that function by clicking in any part of the
  function and pressing :kbd:`F2` in QtCreator.
* Remove the first line containing the following ``RCLCPP_ERROR_STREAM ...`` so
  that the program runs.


Complete Code
-------------

* Find every line that begins with the comment ``Fill Code:`` and read the
  description. Then, replace every instance of the comment ``UNCOMMENT AND COMPLETE``
  with the appropriate line of code.

  .. code-block:: cpp

    /* Fill Code:
        .
        .
        .
    */
    /* UNCOMMENT AND COMPLETE: ... */

* Inspect the ``setAttachedObject`` method to understand how to manipulate a
  ``RobotState`` object which will then be used to construct a motion plan.
* Inspect the ``doMotionPlanning`` method to see how an entire motion plan
  request is defined and sent.
* The |execute()| method sends a motion plan to the robot.


Build Code and Run
------------------

* Compile the pick and place node:

  * In QTCreator: :menuselection:`Build --> Build Project`

  * Alternatively, in a terminal:

    .. code-block:: shell

      colcon build

* Run the supporting nodes with the launch file:

  .. code-block:: shell

    ros2 launch pick_and_place_application application_setup.launch.py

* In another terminal, run your node with the launch file:

  .. code-block:: shell

    ros2 launch pick_and_place_application application_run.launch.py

* The robot should go through the pick moves (approach, pick and retreat) in
  addition to the moves from the previous exercises. In the terminal you should
  see something like this:

  .. code-block:: text

    [INFO] [1400555978.404919764] [pick_and_place_node]: Pick Move 2 Succeeded
    [ERROR] [1400555978.405061541] [pick_and_place_node]: computePlaceToolPoses is not implemented yet.  Aborting.


API References
--------------

* `MoveItCpp class reference <https://docs.ros.org/en/api/moveit_ros_planning/html/classmoveit__cpp_1_1MoveItCpp.html>`_

* |execute()|


.. |execute()| replace:: `execute()`_

.. _execute(): https://docs.ros.org/en/api/moveit_ros_planning/html/classmoveit__cpp_1_1MoveItCpp.html#a5ca934bc472fc16cb8ca62c5263448cd
