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

Application Demo 1 - Perception-Driven Manipulation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

	 Demo 1 - Perception-Driven Manipulation <_source/demo1/index.rst>


Application Demo 2 - Descartes Planning and Execution
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   Demo 2 - Descartes Planning and Execution <_source/demo2/index.rst>

Application Demo 3 - Optimization Based Path Planning
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1
   
   Demo 3 - Optimization Based Path Planning <_source/demo3/index.rst>

Advanced Topics
---------------

Session 5 - Path Planning and Building a Perception Pipeline
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
:download:`Slides <_downloads/slides/ROS-I Advanced Developers Training - Session 5.pdf>`

.. toctree::
   :maxdepth: 1

   Exercise 5.0 - Advanced Descartes Path Planning <_source/session5/Advanced-Descartes-Path-Planning.md>
   Exercise 5.1 - Building a Perception Pipeline <_source/session5/Building-a-Perception-Pipeline.md>
   Exercise 5.2 - Introduction to STOMP <_source/session5/Introduction-to-STOMP.md>
   Exercise 5.3 - Simple PCL Interface for Python <_source/session5/Simple-PCL-Interface-for-Python.rst>
   Exercise 5.4 - OpenCV Image Processing (Python) <_source/session5/OpenCV-in-Python.md>

Session 6 - Documentation, Unit Tests, ROS Utilities and Debugging ROS
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. toctree::
   :maxdepth: 1

   Exercise 6.0 - Documentation Generation <_source/session6/Documentation-Generation.md>
   Exercise 6.1 - Unit Testing <_source/session6/Unit-Testing.rst>
   Exercise 6.2 - Using rqt tools for Analysis <_source/session6/Using-rqt-tools-for-analysis.md>
   Exercise 6.3 - ROS Style Guide and ros_lint <_source/session6/Style-Guide-and-ros_lint.md>
   Exercise 6.4 - Introduction to ROS with Docker and Amazon Web Services (AWS) <_source/session6/Docker-AWS.md>
