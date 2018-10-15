Optimization Based Planning Introduction
========================================

Goal
----

-  The purpose of these exercises is to implement a ROS node that drives
   a robot through a series of moves and actions in order to complete a
   pick and place task. In addition, they will serve as an example of
   how to utilize more specilized tools such as the optimization based
   path planner, TrajOpt, and the control interfaces of the KUKA iiwa robot, while also integrating a variety of software capabilities
   (perception, controller drivers, I/O, inverse kinematics, path
   planning, collision avoidance, etc) into a ROS-based industrial
   application.

   .. image:: /_static/workcell.png
   
Objectives
----------

-  Understand the components and structure of a real or simulated robot
   application.
-  Leverage perception capabilities using PCL
-  Learn how to setup a TrajOpt path planning problem
-  Learn how to use costs and constraints in TrajOpt
-  Learn how to move the arm to a joint or Cartesian position
-  Plan collision-free paths for a pick and place task
-  Send these trajectories to real robot hardware 

Outline
-------

-  First we will explore the given template code and import it into QT
   Creator
-  We will then use the data from a simulated 3D sensor and PCL to find
   the top of the box as the pick point
-  Next, we will give a conceptual introduction to TrajOpt exploring the
   procedure for building a problem, adding costs, and solving for a
   trajectory
-  We will then build a series of helper functions for TrajOpt to
   perform the different parts of a pick and place operation and test in
   simulation
-  With simulation working we will move to a real 3D sensor, learning to
   calibrate it and testing it in conjunction with a simulated robot.
-  Finally, we will move to a physical robot with a real 3D sensor.
