Detect Box Pick Point
=====================

  The coordinate frame of the box's pick can be requested from a ROS service
  that detects it by processing the sensor data. In this exercise, we will
  learn how to use a service client to call that ROS service for the box pick
  pose.


Locate Function
---------------

* In the main program, locate the function call to
  ``application.detect_box_pick()``.
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

* The ``target_recognition_client`` object in your programs can use the
  ``call()`` method to send a request to a ROS service.
* The ROS service that receives the call will process the sensor data and
  return the pose for the box pick in the service structure member
  ``srv.response.target_pose``.


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

* A blue box and voxel grid obstacles will be displayed in rviz. In the
  terminal you should see a message like the following:

  .. code-block:: text

    [ INFO] [1400554224.057842127]: Move wait Succeeded
    [ INFO] [1400554224.311158465]: Gripper opened
    [ INFO] [1400554224.648747043]: target recognition succeeded
    [ERROR] [1400554224.649055043]: create_pick_moves is not implemented yet.  Aborting.


API References
--------------

* `call() <http://docs.ros.org/melodic/api/roscpp/html/classros_1_1ServiceClient.html#a8a0c9be49046998a830df625babd396f>`_
