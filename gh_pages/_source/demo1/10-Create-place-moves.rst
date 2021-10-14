Create Place Moves
==================

  The gripper moves through three poses in order to place the box: Approach,
  place and retreat. In this exercise, we will create these place poses for
  the :abbr:`TCP (Tool Center Point)` coordinate frame.


Locate Function
---------------

* In the main program , locate the function call to
  ``application.computePlaceToolPoses()``.
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
    /* UNCOMMENT AND COMPLETE: */

* The position of the box at the place location is saved in the global variable
  ``cfg.BOX_PLACE_TF``.
* |setOrigin()| and |setRotation()| methods enable the modification of the place transform.
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

* The TCP and wrist positions at the place location will be printed on the
  terminal. You should see something like this:

  .. code-block:: text

    [INFO] [1400556479.404574973] [pick_and_place_node]: Pick Move 2 Succeeded
    [INFO] [1400556479.404866351] [pick_and_place_node]: tcp position at place: [-0.4, 0.6, 0.17]
    [ERROR] [1400556479.404981729] [pick_and_place_node]: doBoxPlace is not implemented yet.  Aborting.


API References
--------------

* |setOrigin()|

* |setRotation()|

.. |setOrigin()| replace:: `setOrigin()`_

.. _setOrigin(): https://docs.ros2.org/foxy/api/tf2/classtf2_1_1Transform.html#ab25fd855dccd651af1a9450ceebe0f00

.. |setRotation()| replace:: `setRotation()`_

.. _setRotation(): https://docs.ros2.org/foxy/api/tf2/classtf2_1_1Transform.html#a1f0d28192f417d4ecde72f88ab5d06a6
