Inspect the ``pick_and_place_application`` Package
==================================================

  In this exercise, we will get familiar with all the files that you will be
  interacting with throughout the next exercises.


Acquire the Workspace
------------------------------------

.. code-block:: shell

  cp -r ~/industrial_training/exercises/Perception-Driven_Manipulation/ros2/template_ws \
  ~/perception_driven_ws


Locate and navigate into the package
------------------------------------

.. code-block:: shell

  cd ~/perception_driven_ws/src/pick_and_place_application/


Look into the ``launch`` directory
----------------------------------

* ``application_setup.launch.py``

  * Brings up the entire ROS system (MoveIt!, Rviz, perception nodes, ROS-I drivers, robot I/O peripherals)

* ``application_run.launch.py``

  * Runs your pick and place node.


Look into the ``config`` directory
----------------------------------

* ``pick_and_place_parameters.yaml``

  * List of parameters read by the pick and place node.

* ``rviz_config.rviz``

  * Rviz configuration file for display properties.

* ``target_recognition_parameters.yaml``

  * Parameters used by the target recognition service for detecting the box from the sensor data.

* ``fake_obstacles_cloud_descriptions.yaml``

  * Parameters used to generate simulated sensor data (simulated sensor mode only).


Look into the ``src`` directory
-------------------------------

* ``nodes/``

  * ``pick_and_place_node.cpp``

    * Main application thread. Contains all necessary headers and function calls.

* ``tasks/``

  * ``create_motion_plan.cpp``
  * ``create_pick_moves.cpp``
  * ``create_place_moves.cpp``
  * ``detect_box_pick.cpp``
  * ``initialize.cpp``
  * ``move_to_wait_position.cpp``
  * ``pickup_box.cpp``
  * ``place_box.cpp``
  * ``reset_world.cpp``
  * ``set_attached_object.cpp``
  * ``set_gripper.cpp``

.. note:: The ``tasks`` directory contains source files with incomplete function definitions. You will fill with code where needed in order to complete the exercise.

* ``utilities/``

  * ``pick_and_place_utilities.cpp``

    * Contains support functions that will help you complete the exercise.
