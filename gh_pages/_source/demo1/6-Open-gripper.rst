Open Gripper
============

  In this exercise, the objective is to use a "grasp action client" to send a
  grasp goal that will open the gripper.


Locate Function
---------------

* In the main program, locate the function call to
  ``application.actuateGripper()``.
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

* The ``grasp_goal.goal`` property can take on three possible values:

  .. code-block:: cpp

    grasp_goal.goal = pick_and_place_msgs::action::ExecuteGraspMove::Goal::GRASP;
    grasp_goal.goal = pick_and_place_msgs::action::ExecuteGraspMove::Goal::RELEASE;
    grasp_goal.goal = pick_and_place_msgs::action::ExecuteGraspMove::Goal::PRE_GRASP;

* Once the grasp flag has been set, you can send the goal through the grasp
  action client. 


Build Code and Run
------------------

* Compile the pick and place node:

  * In QTCreator: :menuselection:`Build --> Build Project`

  * Alternatively, in a terminal:

    .. code-block:: shell

      cd ~/perception_driven_ws
      colcon build

* Run the supporting nodes with the launch file (only if needed):

  .. code-block:: shell

    ros2 launch pick_and_place_application application_setup.launch.py

* In another terminal, run your node with the launch file:

  .. code-block:: shell

    ros2 launch pick_and_place_application application_run.launch.py

* If the task succeeds you will see something like the below in the
  terminal. The robot will not move, only gripper I/O is triggered:

  .. code-block:: text

    [INFO] [1400553290.464877904] [pick_and_place_node]: Move wait Succeeded
    [INFO] [1400553290.720864559] [pick_and_place_node]: Gripper opened
    [ERROR] [1400553290.720985315] [pick_and_place_node]: detectBox is not implemented yet.  Aborting.


API References
--------------

* `async_send_goal() <https://docs.ros2.org/foxy/api/rclcpp_action/classrclcpp__action_1_1Client.html#ae0cf05dc5dee2a1c5d590569b64cba08>`_
* `wait_for() <https://en.cppreference.com/w/cpp/thread/shared_future/wait_for>`_
* `std::chrono::duration <https://en.cppreference.com/w/cpp/chrono/duration>` _

