ROS Industrial Training
===========================================

Welcome to the ROS-Industrial training page!

..
   For developers: the site contents are listed in both normal ReST and in Sphinx toctrees below for
   more flexibility in how the table of contents is presented on the home page. If you make a change
   here, make sure you also make the corresponding change in the toctree directives below which
   define the contents of the sidebar.

Getting Started
---------------

Setup
~~~~~

- :doc:`PC Setup <_source/setup/PC-Setup---ROS>`

Prerequisites
~~~~~~~~~~~~~

C++
***

- `MIT Introduction to C++ <http://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-096-introduction-to-c-january-iap-2011/assignments/>`_
- `Bruce Eckel Thinking in C++ <http://mindview.net/Books/TICPP/ThinkingInCPP2e.html>`_

Linux Fundamentals
******************

- :doc:`0.1 - Intro to Ubuntu GUI <_source/prerequisites/Navigating-the-Ubuntu-GUI>`
- :doc:`0.2 - The Linux File System <_source/prerequisites/Exploring-the-Linux-File-System>`
- :doc:`0.3 - Using the Terminal <_source/prerequisites/The-Linux-Terminal>`

Session 1 - ROS Concepts and Fundamentals (ROS2)
------------------------------------------------

:download:`Slides <_downloads/slides/ROS-I Basic Developers Training - Session 1.pdf>`

- :doc:`1.0 - ROS Setup <_source/session1/ros2/ROS-Setup>`
- :doc:`1.1 - Create a Workspace <_source/session1/ros2/Create-ROS-Workspace>`
- :doc:`1.2 - Installing Packages <_source/session1/ros2/Installing-Existing-Packages>`
- :doc:`1.3 - Packages and Nodes <_source/session1/ros2/Creating-a-ROS-Package-and-Node>`
- :doc:`1.4 - Topics and Messages <_source/session1/ros2/Topics-and-Messages>`

Session 2 - Basic ROS Applications (ROS2)
-----------------------------------------

:download:`Slides <_downloads/slides/ROS-I Basic Developers Training - Session 2.pdf>`

- :doc:`2.0 - Services <_source/session2/ros2/Services>`
- :doc:`2.1 - Actions <_source/session2/ros2/Actions>`
- :doc:`2.2 - Launch Files <_source/session2/ros2/Launch-Files>`
- :doc:`2.3 - Parameters <_source/session2/ros2/Parameters>`

Session 3 - Motion Control of Manipulators
------------------------------------------

:download:`Slides <_downloads/slides/ROS-I Basic Developers Training - Session 3.pdf>`

- :doc:`3.0 - Intro to URDF <_source/session3/Intro-to-URDF>`
- :doc:`3.1 - Workcell XACRO <_source/session3/Workcell-XACRO>`
- :doc:`3.2 - Transforms using TF <_source/session3/Coordinate-Transforms-using-TF>`
- :doc:`3.3 - Build a MoveIt! Package <_source/session3/Build-a-Moveit!-Package>`
- :doc:`3.4 - Motion Planning using RViz <_source/session3/Motion-Planning-RVIZ>`

Session 4 - Descartes and Perception
------------------------------------

:download:`Slides <_downloads/slides/ROS-I Basic Developers Training - Session 4.pdf>`

- :doc:`4.0 - Motion Planning using C++ <_source/session4/Motion-Planning-CPP>`
- :doc:`4.1 - Intro to Descartes <_source/session4/Descartes-Path-Planning>`
- :doc:`4.2 - Intro to Perception <_source/session4/Introduction-to-Perception>`

Application Demos
-----------------

- :doc:`Demo 1 - Perception-Driven Manipulation <_source/demo1/index>`
- :doc:`Demo 2 - Descartes Planning and Execution <_source/demo2/index>`
- :doc:`Demo 3 - Optimization Based Path Planning <_source/demo3/index>`

Advanced Topics
---------------

Session 5 - Path Planning and Perception
----------------------------------------

:download:`Slides <_downloads/slides/ROS-I Advanced Developers Training - Session 5.pdf>`

- :doc:`5.0 - Advanced Descartes Path Planning <_source/session5/Advanced-Descartes-Path-Planning>`
- :doc:`5.1 - Building a Perception Pipeline <_source/session5/Building-a-Perception-Pipeline>`
- :doc:`5.2 - Introduction to STOMP <_source/session5/Introduction-to-STOMP>`
- :doc:`5.3 - Simple PCL Interface for Python <_source/session5/Simple-PCL-Interface-for-Python>`
- :doc:`5.4 - OpenCV Image Processing (Python) <_source/session5/OpenCV-in-Python>`

Session 6 - ROS Tools
---------------------

:download:`Slides <_downloads/slides/ROS-I Advanced Developers Training - Session 5.pdf>`

- :doc:`6.0 - Documentation Generation <_source/session6/Documentation-Generation>`
- :doc:`6.1 - Unit Testing <_source/session6/Unit-Testing>`
- :doc:`6.2 - Using rqt tools for Analysis <_source/session6/Using-rqt-tools-for-analysis>`
- :doc:`6.3 - ROS Style Guide and ros_lint <_source/session6/Style-Guide-and-ros_lint>`
- :doc:`6.4 - Introduction to ROS with Docker and Amazon Web Services (AWS) <_source/session6/Docker-AWS>`

Session 7 - ROS2
----------------

- :doc:`7.0 - ROS2 basics <_source/session7/ROS2-Basics>`
- :doc:`7.1 - ROS1 to ROS2 porting <_source/session7/ROS1-to-ROS2-porting>`
- :doc:`7.2 - Using the ROS1-ROS2 bridge <_source/session7/ROS1-ROS2-bridge>`

Legacy Material
---------------

ROS1 Basics
-----------

- :doc:`1.0 - ROS Setup <_source/session1/ros1/ROS-Setup>`
- :doc:`1.1 - Create a Workspace <_source/session1/ros1/Create-Catkin-Workspace>`
- :doc:`1.2 - Installing Packages <_source/session1/ros1/Installing-Existing-Packages>`
- :doc:`1.3 - Packages and Nodes <_source/session1/ros1/Creating-a-ROS-Package-and-Node>`
- :doc:`1.4 - Topics and Messages <_source/session1/ros1/Topics-and-Messages>`
- :doc:`2.0 - Services <_source/session2/ros1/Services>`
- :doc:`2.1 - Actions <_source/session2/ros1/Actions>`
- :doc:`2.2 - Launch Files <_source/session2/ros1/Launch-Files>`
- :doc:`2.3 - Parameters <_source/session2/ros1/Parameters>`

.. toctree::
    :hidden:
    :caption: Getting Started
    :maxdepth: 1

    PC Setup <_source/setup/PC-Setup---ROS>

    MIT Introduction to C++ <http://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-096-introduction-to-c-january-iap-2011/assignments/>
    Bruce Eckel Thinking in C++ <http://mindview.net/Books/TICPP/ThinkingInCPP2e.html>

    0.1 - Intro to Ubuntu GUI <_source/prerequisites/Navigating-the-Ubuntu-GUI>
    0.2 - The Linux File System <_source/prerequisites/Exploring-the-Linux-File-System>
    0.3 - Using the Terminal <_source/prerequisites/The-Linux-Terminal>

.. toctree::
    :hidden:
    :caption: ROS Basics (ROS2)
    :maxdepth: 1

    1.0 - ROS Setup <_source/session1/ros2/ROS-Setup>
    1.1 - Create a Workspace <_source/session1/ros2/Create-ROS-Workspace>
    1.2 - Installing Packages <_source/session1/ros2/Installing-Existing-Packages>
    1.3 - Packages and Nodes <_source/session1/ros2/Creating-a-ROS-Package-and-Node>
    1.4 - Topics and Messages <_source/session1/ros2/Topics-and-Messages>
    2.0 - Services <_source/session2/ros2/Services>
    2.1 - Actions <_source/session2/ros2/Actions>
    2.2 - Launch Files <_source/session2/ros2/Launch-Files>
    2.3 - Parameters <_source/session2/ros2/Parameters>

.. toctree::
    :hidden:
    :caption: ROS-Industrial Basics
    :maxdepth: 1

    3.0 - Intro to URDF <_source/session3/Intro-to-URDF>
    3.1 - Workcell XACRO <_source/session3/Workcell-XACRO>
    3.2 - Transforms using TF <_source/session3/Coordinate-Transforms-using-TF>
    3.3 - Build a MoveIt! Package <_source/session3/Build-a-Moveit!-Package>
    3.4 - Motion Planning using RViz <_source/session3/Motion-Planning-RVIZ>
    4.0 - Motion Planning using C++ <_source/session4/Motion-Planning-CPP>
    4.1 - Intro to Descartes <_source/session4/Descartes-Path-Planning>
    4.2 - Intro to Perception <_source/session4/Introduction-to-Perception>

.. toctree::
    :hidden:
    :caption: Application Demos
    :maxdepth: 1

    Demo 1 - Perception-Driven Manipulation <_source/demo1/index>
    Demo 2 - Descartes Planning and Execution <_source/demo2/index>
    Demo 3 - Optimization Based Path Planning <_source/demo3/index>

.. toctree::
    :hidden:
    :caption: Advanced Topics
    :maxdepth: 1

    5.0 - Advanced Descartes Path Planning <_source/session5/Advanced-Descartes-Path-Planning>
    5.1 - Building a Perception Pipeline <_source/session5/Building-a-Perception-Pipeline>
    5.2 - Introduction to STOMP <_source/session5/Introduction-to-STOMP>
    5.3 - Simple PCL Interface for Python <_source/session5/Simple-PCL-Interface-for-Python>
    5.4 - OpenCV Image Processing (Python) <_source/session5/OpenCV-in-Python>
    6.0 - Documentation Generation <_source/session6/Documentation-Generation>
    6.1 - Unit Testing <_source/session6/Unit-Testing>
    6.2 - Using rqt tools for Analysis <_source/session6/Using-rqt-tools-for-analysis>
    6.3 - ROS Style Guide and ros_lint <_source/session6/Style-Guide-and-ros_lint>
    6.4 - Introduction to ROS with Docker and Amazon Web Services (AWS) <_source/session6/Docker-AWS>
    7.0 - ROS2 basics <_source/session7/ROS2-Basics>
    7.1 - ROS1 to ROS2 porting <_source/session7/ROS1-to-ROS2-porting>
    7.2 - Using the ROS1-ROS2 bridge <_source/session7/ROS1-ROS2-bridge>

.. toctree::
    :hidden:
    :caption: Legacy Material (ROS1)
    :maxdepth: 1

    1.0 - ROS Setup <_source/session1/ros1/ROS-Setup>
    1.1 - Create a Workspace <_source/session1/ros1/Create-Catkin-Workspace>
    1.2 - Installing Packages <_source/session1/ros1/Installing-Existing-Packages>
    1.3 - Packages and Nodes <_source/session1/ros1/Creating-a-ROS-Package-and-Node>
    1.4 - Topics and Messages <_source/session1/ros1/Topics-and-Messages>
    2.0 - Services <_source/session2/ros1/Services>
    2.1 - Actions <_source/session2/ros1/Actions>
    2.2 - Launch Files <_source/session2/ros1/Launch-Files>
    2.3 - Parameters <_source/session2/ros1/Parameters>
