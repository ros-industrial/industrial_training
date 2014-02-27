industrial_training
===================

This branch was created in order to address the issue of attached objects not being checked for collisions agains sensor data <octomap>

Setup:
  - add the 'industrial_training/training/supplements' and 'industrial_training/training/ref/demo_manipulation' directories to the ROS_PACKAGE_PATH variable
  - roscd into collision_avoidance_pick_and_place and run rosmake

Run Demo in this order:
  - Terminal 1: roslaunch collision_avoidance_pick_and_place ur5_setup.launch
  - Terminal 2: roslaunch collision_avoidance_pick_and_place ur5_pick_and_place.launch

Once running the arm will pick and place a box while avoiding collisions with the sensor data.  The box will go right through the obstacles (octomap) in most cases.

The launch flie from the second terminal needs to be restarted in order for the arm to run once again. The launch file from the first terminal may continue to run permanently.
