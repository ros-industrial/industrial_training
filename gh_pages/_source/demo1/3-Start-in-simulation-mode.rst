Start in Simulation Mode
========================

  In this exercise, we will start a ROS system that is ready to move the robot in simulation mode.


Run setup launch file in simulation mode (simulated robot and sensor)
---------------------------------------------------------------------

In a terminal enter:

.. code-block:: shell

  source ~/perception_driven_ws/install/setup.bash
  ros2 launch pick_and_place_application application_setup.launch.py

Rviz will display all the workcell components including the robot in its default position; at this stage your system is ready. However, no motion will take place until we run the pick and place node.


Setup for real robot and sensor
-----------------------------------------

If you have the equipment and want to run this demo on real hardware, you need to pass some 
additional arguments to the launch file.

.. code-block:: shell

  ros2 launch pick_and_place_application application_setup.launch.py \
      use_sim_robot:=false robot_ip:={robot IP address}
