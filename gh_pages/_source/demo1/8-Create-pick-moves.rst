Create Pick Moves
=================

  The gripper moves through three poses in order to do a pick: approach,
  target and retreat. In this exercise, we will use the box pick transform to
  create the pick poses for the :abbr:`TCP (Tool Center Point)` coordinate
  frame.


Locate Function
---------------

* In the main program, locate the function call to
  ``application.computePickToolPoses()``.
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

* We want to set where the box currently is as the pick location with |setOrigin()|.
* After setting the right location, |setRotation()| makes the end-effector face
  the correct direction.
* The ``createManipulationPoses()`` uses the values of the approach and
  retreat distances in order to create the corresponding poses at the desired
  target.

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

* The TCP and wrist positions at the pick will be printed in the terminal. You
  should see something like this:

  .. code-block:: text

    [INFO] [1400555434.918332145] [pick_and_place_node]: Move wait Succeeded
    [INFO] [1400555435.172714267] [pick_and_place_node]: Gripper opened
    [INFO] [1400555435.424279410] [pick_and_place_node]: target recognition succeeded
    [INFO] [1400555435.424848964] [pick_and_place_node]: tcp position at pick: [-0.8, 0.2, 0.17]
    [INFO] [1400555435.424912520] [pick_and_place_node]: tcp z direction at pick: [8.65611e-17, -8.66301e-17, -1]
    [ERROR] [1400555435.425051853] [pick_and_place_node]: doBoxPickup is not implemented yet.  Aborting.


API References
--------------

* |setOrigin()|

* |setRotation()|

.. |setOrigin()| replace:: `setOrigin()`_

.. _setOrigin(): https://docs.ros2.org/foxy/api/tf2/classtf2_1_1Transform.html#ab25fd855dccd651af1a9450ceebe0f00

.. |setRotation()| replace:: `setRotation()`_

.. _setRotation(): https://docs.ros2.org/foxy/api/tf2/classtf2_1_1Transform.html#a1f0d28192f417d4ecde72f88ab5d06a6
