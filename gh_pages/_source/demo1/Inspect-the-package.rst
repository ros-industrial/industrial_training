Inspect the ``pick_and_place_exercise`` Package
===============================================

  In this exercise, we will get familiar with all the files that you'll be
  interacting with throughout these exercises. 


Acquire and initialize the Workspace
------------------------------------

.. code-block:: shell

  cp -r ~/industrial_training/exercises/Perception-Driven_Manipulation/\
  template_ws ~/perception_driven_ws
  cd ~/perception_driven_ws
  source /opt/ros/melodic/setup.bash
  catkin init
  wstool update -t src


Download debian dependencies
----------------------------

.. warning:: Make sure you have installed and configured the `rosdep tool <http://wiki.ros.org/rosdep>`_.

Then, run the following command from the :file:`src` directory of your workspace:

.. code-block:: shell
  
  rosdep install --from-paths . --ignore-src -y


Build your workspace
--------------------

.. code-block:: shell

  catkin build

.. note:: If the build fails then revisit the previous two steps to make sure all the dependencies were downloaded.


Source the workspace
--------------------

Run the following command from your workspace parent directory

.. code-block:: shell

  source devel/setup.bash


Locate and navigate into the package
------------------------------------

.. code-block:: shell

  cd ~/perception_driven_ws/src/collision_avoidance_pick_and_place/


Look into each file in the launch directory
-------------------------------------------

* :file:`ur5_setup.launch`: Brings up the entire ROS system (MoveIt!, rviz,
  perception, ROS-I drivers, robot I/O peripherals)
* :file:`ur5_pick_and_place.launch`: Runs your pick and place node.


Look into the config directory
------------------------------

* :file:`ur5/`

  * :file:`pick_and_place_parameters.yaml`: List of parameters read by the pick
    and place node.
  * :file:`rviz_config.rviz`: Rviz configuration file for display properties.
  * :file:`target_recognition_parameters.yaml`: Parameters used by the target
    recognition service for detecting the box from the sensor data.
  * :file:`test_cloud_obstacle_descriptions.yaml`: Parameters used to generate
    simulated sensor data (simulated sensor mode only).
  * :file:`collision_obstacles.txt`: Description of each obstacle blob added
    to the simulated sensor data (simulated sensor mode only)


Look into the src directory
---------------------------

* :file:`nodes/`

  * :file:`pick_and_place_node.cpp`: Main application thread. Contains all
    necessary headers and function calls.

* :file:`tasks/`: Source files with incomplete function definitions. You will
  fill with code where needed in order to complete the exercise.

  * :file:`create_motion_plan.cpp`
  * :file:`create_pick_moves.cpp`
  * :file:`create_place_moves.cpp`
  * :file:`detect_box_pick.cpp`
  * :file:`pickup_box.cpp`
  * :file:`place_box.cpp`
  * :file:`move_to_wait_position.cpp`
  * :file:`set_attached_object.cpp`
  * :file:`set_gripper.cpp`

* :file:`utilities/`
 
  * :file:`pick_and_place_utilities.cpp`: Contains support functions that will
    help you complete the exercise.
