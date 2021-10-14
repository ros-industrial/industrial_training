Move Arm to Wait Position
=========================

  MoveIt!'s ``moveit_cpp`` API allows us to move the robot in various
  ways.  With a ``PlanningComponent`` object, it is possible to 
  create motion plans to move to a desired joint position, Cartesian goal or a 
  predefined pose created with the Setup Assistant. Then we can use a 
  ``MoveItCpp`` object to execute that plan. In this exercise, we 
  will move the robot to a predefined joint pose.

Locate Function
---------------

* In the main program, locate the method call to
  ``application.moveToWaitPosition()``.
* Go to the source file of that function by clicking in any part of the
  function and pressing :kbd:`F2` in QtCreator.
* Alternatively, browse to the file at

  .. code-block:: shell

    [workspace source directory]/pick_and_place_application/src/tasks/move_to_wait_position.cpp

* Remove the first line containing the following ``RCLCPP_ERROR ...`` so
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

* The name of the predefined "wait" pose was saved in the global variable
  ``cfg.WAIT_POSE_NAME`` during initialization.
* Calling |plan()| provides a solution trajectory stored in
  a ``PlanSolution`` object
* The |execute()| method sends a motion plan to the robot.

Build Code and Run
------------------

* Compile the pick and place node:

  * In QTCreator: :menuselection:`Build --> Build Project`
  * Alternatively, in a terminal:

    .. code-block:: shell

      cd ~/perception_driven_ws
      colcon build

* In a terminal, launch the supporting nodes:

  .. code-block:: shell

    source ~/perception_driven_ws/install/setup.bash
    ros2 launch pick_and_place_application application_setup.launch.py

* In another terminal, launch the application node:

  .. code-block:: shell

    source ~/perception_driven_ws/install/setup.bash
    ros2 launch pick_and_place_application application_run.launch.py

* If the robot is not already in the wait position, it should move to it.

In the terminal, you should see something like this:

.. code-block:: text

  [INFO] [1400553673.460328538] [pick_and_place_node]: Move wait Succeeded
  [ERROR] [1400553673.460434627] [pick_and_place_node]: actuateGripper is not implemented yet. Aborting.

API References
--------------

* `setGoal() <https://docs.ros.org/en/api/moveit_ros_planning/html/classmoveit__cpp_1_1PlanningComponent.html#aa89cf6ec7cf184c07fd78e3ed1a39c5a>`_
* |plan()|
* |execute()|

.. |plan()| replace:: `plan()`_

.. _plan(): https://docs.ros.org/en/api/moveit_ros_planning/html/classmoveit__cpp_1_1PlanningComponent.html#a66046e476ffb45002432c9020ff0a91f

.. |execute()| replace:: `execute()`_

.. _execute(): https://docs.ros.org/en/api/moveit_ros_planning/html/classmoveit__cpp_1_1MoveItCpp.html#a5ca934bc472fc16cb8ca62c5263448cd

