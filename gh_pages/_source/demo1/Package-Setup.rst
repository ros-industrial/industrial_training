Package Setup
=============

  In this exercise, we'll build our package dependencies and configure the
  package for the Qt Creator IDE.


Build Package Dependencies
--------------------------

In a terminal enter:

.. code-block:: shell

  cd ~/perception_driven_ws
  catkin build
  source devel/setup.bash


Import Package into QTCreator
-----------------------------

In QTCreator select the following menu item:
:menuselection:`File --> New File or Project`.

In the dialog that appears, on the left select :guilabel:`Other Project` and
in the middle section select :guilabel:`ROS Workspace`. Confirm your selection
with the :guilabel:`Choose` button.


Open the Main Thread Source File
--------------------------------

In the :guilabel:`Edit` tab, open the file :file:`pick_and_place_node.cpp` in the directory ``[workspace source directory]/collision_avoidance_pick_and_place/src/nodes``
