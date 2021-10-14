Detect Box Pick Point
=====================

  The coordinate frame of the box's pick can be requested from a ROS service
  that detects it by processing the sensor data. In this exercise, we will
  learn how to use a service client to call that ROS service for the box pick
  pose.


Locate Function
---------------

* In the main program, locate the function call to
  ``application.detectBox()``.
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

* The ``target_recognition_client`` object in your programs can use the
  ``async_send_request()`` method to send a request to a ROS service.
* The ROS service that receives the request will process the sensor data and
  return the pose for the box pick. This response can be retrieved by calling ``get()``
  on ``response_fut``. The pose can then be accessed with ``response->target_pose``.


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

* A blue box and voxel grid obstacles will be displayed in rviz. In the
  terminal you should see something like this:

  .. code-block:: text

    [INFO] [1400554224.057842127] [pick_and_place_node]: Move wait Succeeded
    [INFO] [1400554224.311158465] [pick_and_place_node]: Gripper opened
    [INFO] [1400554224.648747043] [pick_and_place_node]: target recognition succeeded
    [ERROR] [1400554224.649055043] [pick_and_place_node]: computePickToolPoses is not implemented yet.  Aborting.

API References
--------------

* `async_send_request() <https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Client.html#a7567297f43b72f96e8ec57fa7ff2f4e1>`_
