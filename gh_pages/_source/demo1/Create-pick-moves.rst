Create Pick Moves
=================

  The gripper moves through three poses in order to do a pick: approach,
  target and retreat. In this exercise, we will use the box pick transform to
  create the pick poses for the :abbr:`TCP (Tool Center Point)` coordinate
  frame and then transform them to the arm's wrist coordinate frame.


Locate Function
---------------

* In the main program, locate the function call to
  ``application.create_pick_moves()``.
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

* The ``create_manipulation_poses()`` uses the values of the approach and
  retreat distances in order to create the corresponding poses at the desired
  target.
* Since MoveIt! plans the robot path for the arm's wrist, it is necessary to
  convert all the pick poses to the wrist coordinate frame.
* The |lookupTransform()|_ method can provide the pose of a target relative to
  another pose.


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

* The tcp and wrist position at the pick will be printed in the terminal. You
  should see something like this:

  .. code-block:: text

    [ INFO] [1400555434.918332145]: Move wait Succeeded
    [ INFO] [1400555435.172714267]: Gripper opened
    [ INFO] [1400555435.424279410]: target recognition succeeded
    [ INFO] [1400555435.424848964]: tcp position at pick: [-0.8, 0.2, 0.17]
    [ INFO] [1400555435.424912520]: tcp z direction at pick: [8.65611e-17, -8.66301e-17, -1]
    [ INFO] [1400555435.424993675]: wrist position at pick: x: -0.81555
    y: 0.215563
    z: 0.3

    [ERROR] [1400555435.425051853]: pickup_box is not implemented yet.  Aborting.


API References
--------------

* |lookupTransform()|

* `TF Transforms and other useful data types <http://wiki.ros.org/tf/Overview/Data%20Types>`_


.. |lookupTransform()| replace:: `lookupTransform()`_

.. _lookupTransform(): http://docs.ros.org/melodic/api/tf/html/c++/classtf_1_1Transformer.html#a14536fe915c0c702534409c15714aa2f
