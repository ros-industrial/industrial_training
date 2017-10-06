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

   PC Setup <_source/setup/PC-Setup---ROS-Kinetic.md>

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
:download:`Slides <_downloads/slides/ROS-I Basic Developers Training - Session 0.pdf>`

.. toctree::
   :maxdepth: 1

   Exercise 0.1 - Intro to Ubuntu GUI <_source/prerequisites/Navigating-the-Ubuntu-GUI.md>
   Exercise 0.2 - The Linux File System <_source/prerequisites/Exploring-the-Linux-File-System.md>
   Exercise 0.3 - Using the Terminal <_source/prerequisites/The-Linux-Terminal.md>

Introduction
------------
:download:`Slides <_downloads/slides/ROS-I Developers Training - Introduction.pdf>`

Basic Topics
------------

Session 1 - ROS Concepts and Fundamentals
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
:download:`Slides <_downloads/slides/ROS-I Basic Developers Training - Session 1.pdf>`

.. toctree::
   :maxdepth: 1

   Exercise 1.0 - ROS Setup <_source/session1/ROS-Setup.md>
   Exercise 1.1 - Create a Workspace <_source/session1/Create-Catkin-Workspace.md>
   Exercise 1.2 - Installing Packages <_source/session1/Installing-Existing-Packages.md>
   Exercise 1.3 - Packages and Nodes <_source/session1/Creating-a-ROS-Package-and-Node.md>
   Exercise 1.4 - Topics and Messages <_source/session1/Topics-and-Messages.md>

Session 2 - Basic ROS Applications
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
:download:`Slides <_downloads/slides/ROS-I Basic Developers Training - Session 2.pdf>`

.. toctree::
   :maxdepth: 1

   Exercise 2.0 - Services <_source/session2/Services.md>
   Exercise 2.1 - Actions <_source/session2/Actions.md>
   Exercise 2.2 - Launch Files <_source/session2/Launch-Files.md>
   Exercise 2.3 - Parameters <_source/session2/Parameters.md>

Session 3 - Motion Control of Manipulators
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
:download:`Slides <_downloads/slides/ROS-I Basic Developers Training - Session 3.pdf>`

.. toctree::
   :maxdepth: 1

   Exercise 3.0 - Intro to URDF <_source/session3/Intro-to-URDF.md>
   Exercise 3.1 - Workcell XACRO <_source/session3/Workcell-XACRO.md>
   Exercise 3.2 - Transforms using TF <_source/session3/Coordinate-Transforms-using-TF.md>
   Exercise 3.3 - Build a MoveIt! Package <_source/session3/Build-a-Moveit!-Package.md>
   Exercise 3.4 - Motion Planning using RViz <_source/session3/Motion-Planning-RVIZ.md>

Session 4 - Descartes and Perception
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
:download:`Slides <_downloads/slides/ROS-I Basic Developers Training - Session 4.pdf>`

.. toctree::
   :maxdepth: 1

   Exercise 4.0 - Motion Planning using C++ <_source/session4/Motion-Planning-CPP.md>
   Exercise 4.1 - Intro to Descartes <_source/session4/Descartes-Path-Planning.md>
   Exercise 4.2 - Intro to Perception <_source/session4/Introduction-to-Perception.md>

Application Demo - Perception-Driven Manipulation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   Demo 1.0 - Introduction <_source/demo1/Introduction.md>
   Demo 1.1 - Inspect the "pick_and_place_exercise" Package <_source/demo1/Inspect-the-package.md>
   Demo 1.2 - Package Setup <_source/demo1/Package-Setup.md>
   Demo 1.3 - Start in Simulation Mode <_source/demo1/Bring-up-ROS-system-in-simulation-mode.md>
   Demo 1.4 - Initialization and Global Variables <_source/demo1/Initialization-and-global-variables.md>
   Demo 1.5 - Move Arm to Wait Position <_source/demo1/Move-arm-to-wait-position.md>
   Demo 1.6 - Open Gripper <_source/demo1/Open-gripper.md>
   Demo 1.7 - Detect Box Pick Point <_source/demo1/Detect-box-pick-point.md>
   Demo 1.8 - Create Pick Moves <_source/demo1/Create-pick-moves.md>
   Demo 1.9 - Pick Up Box <_source/demo1/Pick-up-box.md>
   Demo 1.10 - Create Place Moves <_source/demo1/Create-place-moves.md>
   Demo 1.11 - Place Box <_source/demo1/Place-box.md>

Application Demo - Descartes Planning and Execution
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   Demo 2.0 - Introduction <_source/demo2/Introduction.md>
   Demo 2.1 - Application Structure <_source/demo2/Application-Structure.md>
   Demo 2.2 - General Instructions <_source/demo2/General-Instructions.md>
   Demo 2.3 - Load Parameters <_source/demo2/Load-Parameters.md>
   Demo 2.4 - Initialize ROS <_source/demo2/Initialize-ROS.md>
   Demo 2.5 - Initialize Descartes <_source/demo2/Initialize-Descartes.md>
   Demo 2.6 - Move Home <_source/demo2/Move-Home.md>
   Demo 2.7 - Generate a Semi-Constrained Trajectory <_source/demo2/Generate-a-Semi-Constrained-Trajectory.md>
   Demo 2.8 - Plan a Robot Path <_source/demo2/Plan-a-robot-path.md>
   Demo 2.9 - Run a Robot Path <_source/demo2/Run-a-robot-path.md>

Advanced Topics
---------------

Session 5 - Path Planning and Building a Perception Pipeline
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
:download:`Slides <_downloads/slides/ROS-I Advacned Developers Training - Session 5.pdf>`

.. toctree::
   :maxdepth: 1

   Exercise 5.0 - Advanced Descartes Path Planning <_source/session5/Advanced-Descartes-Path-Planning.md>
   Exercise 5.1 - Building a Perception Pipeline <_source/session5/Building-a-Perception-Pipeline.md>
   Exercise 5.2 - Introduction to STOMP <_source/session5/Introduction-to-STOMP.md>

Session 6 - Documentation, Unit Tests, ROS Utilities and Debugging ROS
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
:download:`Slides <_downloads/slides/ROS-I Advacned Developers Training - Session 5.pdf>`

.. toctree::
   :maxdepth: 1

   Exercise 6.0 - Documentation Generation <_source/session6/Documentation-Generation.md>
   Exercise 6.1 - Unit Testion <_source/session6/Unit-Testing.md>
   Exercise 6.2 - Using rqt tools for Analysis <_source/session6/Using-rqt-tools-for-analysis.md>
   Exercise 6.3 - ROS Style Guide and ros_lint <_source/session6/Style-Guide-and-ros_lint.md>
   Exercise 6.4 - Introduction to ROS with Docker and Amazon Web Services (AWS) <_source/session6/Docker-AWS.md>
