Package Setup
=============

  In this exercise, we will build our package dependencies and configure the
  package for the Qt Creator IDE.


Download dependencies from source
---------------------------------

In a terminal enter:

.. code-block:: shell

  cd ~/perception_driven_ws/src
  vcs import < dependencies.repos


Download debian dependencies
----------------------------

.. note:: Make sure you installed and configured the `rosdep tool <http://wiki.ros.org/rosdep>`_.

Then, run the following command from the :file:`src` directory of your workspace:

.. code-block:: shell

  rosdep install --from-paths . --ignore-src -y


Build your workspace
--------------------

.. code-block:: shell

  cd ~/perception_driven_ws
  source /opt/ros/foxy/setup.bash
  colcon build

.. note:: If the build fails then revisit the previous two steps to make sure all the dependencies were downloaded.


Source the workspace
--------------------

In a new terminal (where you didn't build the workspace), run the following command from your workspace parent directory to enable the ROS tools to see the new packages

.. code-block:: shell

  source install/setup.bash


Import Package into QTCreator
-----------------------------

In QTCreator select the following menu item:
:menuselection:`File --> New File or Project`.

In the dialog that appears, on the left select :guilabel:`Other Project` and
in the middle section select :guilabel:`ROS Workspace`. Confirm your selection
with the :guilabel:`Choose` button.

Choose a name for your workspace, select ``/opt/ros/foxy/`` as the ROS distribution,
select ``colcon`` as the build system, and provide the path to your ``perception_ws``
directory as the workspace path. Click :guilabel:`Next` and :guilabel:`Finish` to 
complete the setup.

Rebuild the workspace inside of QTCreator, to create the metadata required for IDE code completion.


Open the Main Thread Source File
--------------------------------

In the :guilabel:`Edit` tab, open the file :file:`pick_and_place_node.cpp` in the directory ``[workspace source directory]/pick_and_place_application/src/nodes``
