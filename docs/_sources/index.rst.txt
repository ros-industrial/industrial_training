.. SphinxTest documentation master file, created by
   sphinx-quickstart on Tue Oct  3 11:09:13 2017.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

ROS Industrial (Kinetic) Training Exercises
===========================================

Setup PC
--------

.. toctree::
   :maxdepth: 1

   PC Setup <doc/kinetic/setup/PC-Setup---ROS-Kinetic.md>

Prerequisites
-------------

C++
~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   MIT Introduction to C++ <http://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-096-introduction-to-c-january-iap-2011/assignments/>
   Bruce Eckel Thinking in C++ <http://mindview.net/Books/TICPP/ThinkingInCPP2e.html>


Linux Fundamentals
~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   Exercise 0.1 - Intro to Ubuntu GUI <doc/kinetic/prerequisites/Navigating-the-Ubuntu-GUI.md>
   Exercise 0.2 - The Linux File System <doc/kinetic/prerequisites/Exploring-the-Linux-File-System.md>
   Exercise 0.3 - Using the Terminal <doc/kinetic/prerequisites/The-Linux-Terminal.md>


Basic Topics
------------

Session 1 - ROS Concepts and Fundamentals
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   Exercise 1.0 - ROS Setup <doc/kinetic/session1/ROS-Setup.md>
   Exercise 1.1 - Create a Workspace <doc/kinetic/session1/Create-Catkin-Workspace.md>
   Exercise 1.2 - Installing Packages <doc/kinetic/session1/Installing-Existing-Packages.md>
   Exercise 1.3 - Packages and Nodes <doc/kinetic/session1/Creating-a-ROS-Package-and-Node.md>
   Exercise 1.4 - Topics and Messages <doc/kinetic/session1/Topics-and-Messages.md>

Session 2 - Basic ROS Applications
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   Exercise 2.0 - Services <doc/kinetic/session2/Services.md>
   Exercise 2.1 - Actions <doc/kinetic/session2/Actions.md>
   Exercise 2.2 - Launch Files <doc/kinetic/session2/Launch-Files.md>
   Exercise 2.3 - Parameters <doc/kinetic/session2/Parameters.md>

Session 3 - Motion Control of Manipulators
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   Exercise 3.0 - Intro to URDF <doc/kinetic/session3/Intro-to-URDF.md>
   Exercise 3.1 - Workcell XACRO <doc/kinetic/session3/Workcell-XACRO.md>
   Exercise 3.2 - Transforms using TF <doc/kinetic/session3/Coordinate-Transforms-using-TF.md>
   Exercise 3.3 - Build a MoveIt! Package <doc/kinetic/session3/Build-a-Moveit!-Package.md>
   Exercise 3.4 - Motion Planning using RViz <doc/kinetic/session3/Motion-Planning-RVIZ.md>

Session 4 - Descartes and Perception
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   Exercise 4.0 - Motion Planning using C++ <doc/kinetic/session4/Motion-Planning-CPP.md>
   Exercise 4.1 - Intro to Descartes <doc/kinetic/session4/Descartes-Path-Planning.md>
   Exercise 4.2 - Intro to Perception <doc/kinetic/session4/Introduction-to-Perception.md>

Application Demo - Perception-Driven Manipulation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   Demo 1.0 - Introduction <doc/kinetic/demo1/Introduction.md>
   Demo 1.1 - Inspect the "pick_and_place_exercise" Package <doc/kinetic/demo1/Inspect-the-package.md>
   Demo 1.2 - Package Setup <doc/kinetic/demo1/Package-Setup.md>
   Demo 1.3 - Start in Simulation Mode <doc/kinetic/demo1/Bring-up-ROS-system-in-simulation-mode.md>
   Demo 1.4 - Initialization and Global Variables <doc/kinetic/demo1/Initialization-and-global-variables.md>
   Demo 1.5 - Move Arm to Wait Position <doc/kinetic/demo1/Move-arm-to-wait-position.md>
   Demo 1.6 - Open Gripper <doc/kinetic/demo1/Open-gripper.md>
   Demo 1.7 - Detect Box Pick Point <doc/kinetic/demo1/Detect-box-pick-point.md>
   Demo 1.8 - Create Pick Moves <doc/kinetic/demo1/Create-pick-moves.md>
   Demo 1.9 - Pick Up Box <doc/kinetic/demo1/Pick-up-box.md>
   Demo 1.10 - Create Place Moves <doc/kinetic/demo1/Create-place-moves.md>
   Demo 1.11 - Place Box <doc/kinetic/demo1/Place-box.md>

Application Demo - Descartes Planning and Execution
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   Demo 2.0 - Introduction <doc/kinetic/demo2/Introduction.md>
   Demo 2.1 - Application Structure <doc/kinetic/demo2/Application-Structure.md>
   Demo 2.2 - General Instructions <doc/kinetic/demo2/General-Instructions.md>
   Demo 2.3 - Load Parameters <doc/kinetic/demo2/Load-Parameters.md>
   Demo 2.4 - Initialize ROS <doc/kinetic/demo2/Initialize-ROS.md>
   Demo 2.5 - Initialize Descartes <doc/kinetic/demo2/Initialize-Descartes.md>
   Demo 2.6 - Move Home <doc/kinetic/demo2/Move-Home.md>
   Demo 2.7 - Generate a Semi-Constrained Trajectory <doc/kinetic/demo2/Generate-a-Semi-Constrained-Trajectory.md>
   Demo 2.8 - Plan a Robot Path <doc/kinetic/demo2/Plan-a-robot-path.md>
   Demo 2.9 - Run a Robot Path <doc/kinetic/demo2/Run-a-robot-path.md>

Advanced Topics
---------------

Session 5 - Path Planning and Building a Perception Pipeline
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   Exercise 5.0 - Advanced Descartes Path Planning <doc/kinetic/session5/Advanced-Descartes-Path-Planning.md>
   Exercise 5.1 - Building a Perception Pipeline <doc/kinetic/session5/Building-a-Perception-Pipeline.md>
   Exercise 5.2 - Introduction to STOMP <doc/kinetic/session5/Introduction-to-STOMP.md>

Session 6 - Documentation, Unit Tests, ROS Utilities and Debugging ROS
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   Exercise 6.0 - Documentation Generation <doc/kinetic/session6/Documentation-Generation.md>
   Exercise 6.1 - Unit Testion <doc/kinetic/session6/Unit-Testing.md>
   Exercise 6.2 - Using rqt tools for Analysis <doc/kinetic/session6/Using-rqt-tools-for-analysis.md>
   Exercise 6.3 - ROS Style Guide and ros_lint <doc/kinetic/session6/Style-Guide-and-ros_lint.md>
   Exercise 6.4 - Introduction to ROS with Docker and Amazon Web Services (AWS) <doc/kinetic/session6/Docker-AWS.md>
