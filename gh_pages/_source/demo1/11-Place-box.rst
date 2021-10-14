Place Box
=========

  In this exercise, we will move the robot through the pick and place motions 
  while avoiding obstacles with an attached payload. In addition, the gripper 
  must be opened or closed at the appropriate time in order to complete the 
  task.


Locate Function
---------------

* In the main program, locate the function call to ``application.doBoxPlace()``.
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

* The |execute()| method sends a motion plan to the robot.


Build Code and Run
------------------

* Compile the pick and place node:

  * In QTCreator: :menuselection:`Build --> Build Project`

  * Alternatively, in a terminal:

    .. code-block:: shell

      colcon build

* Run the supporting nodes with the launch file (only if needed):

  .. code-block:: shell

    ros2 launch pick_and_place_application application_setup.launch.py

* In another terminal, run your node with the launch file:

  .. code-block:: shell

    ros2 launch pick_and_place_application application_run.launch.py

* At this point your exercise is complete and the robot should move through
  the pick and place motions and then back to the wait pose. Congratulations!


API References
--------------

* `MoveItCpp class reference <https://docs.ros.org/en/api/moveit_ros_planning/html/classmoveit__cpp_1_1MoveItCpp.html>`_

* |execute()|_


.. |execute()| replace:: `execute()`_

.. _execute(): https://docs.ros.org/en/api/moveit_ros_planning/html/classmoveit__cpp_1_1MoveItCpp.html#a5ca934bc472fc16cb8ca62c5263448cd
