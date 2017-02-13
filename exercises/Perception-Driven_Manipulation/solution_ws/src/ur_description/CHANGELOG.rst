^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.2 (2014-03-31)
------------------

1.0.1 (2014-03-31)
------------------
* changes due to file renaming
* generate urdfs from latest xacros
* file renaming
* adapt launch files in order to be able to use normal/limited xacro
* fixed typo in limits
* add joint_limited urdf.xacros for both robots
* (re-)add ee_link for both robots
* updates for latest gazebo under hydro
* remove ee_link - as in ur10
* use same xacro params as ur10
* use new transmission interfaces
* update xml namespaces for hydro
* remove obsolete urdf file
* remove obsolete urdf file
* Contributors: ipa-fxm

* Update ur10.urdf.xacro
  Corrected UR10's urdf to faithfully represent joint effort thresholds, velocity limits, and dynamics parameters.
* Update ur5.urdf.xacro
  Corrected effort thresholds and friction values for UR5 urdf.
* added corrected mesh file
* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Corrected warning on xacro-files in hydro.
* Added definitions for adding tergets in install folder. Issue `#10 <https://github.com/ros-industrial/universal_robot/issues/10>`_.
* Updated to catkin.  ur_driver's files were added to nested Python directory for including in other packages.
* fixed name of ur5 transmissions
* patched gazebo.urdf.xacro to be compatible with gazebo 1.5
* fixed copy&paste error (?)
* prefix versions of gazebo and transmission macros
* Added joint limited urdf and associated moveit package.  The joint limited package is friendlier to the default KLD IK solution
* Added ur5 moveit library.  The Kinematics used by the ur5 move it library is unreliable and should be replaced with the ur_kinematics
* Updated urdf files use collision/visual models.
* Reorganized meshes to include both collision and visual messhes (like other ROS-I robots).  Modified urdf xacro to include new models.  Removed extra robot pedestal link from urdf (urdfs should only include the robot itself).
* minor changes on ur5 xacro files
* Removed extra stl files and fixed indentions
* Renamed packages and new groovy version
* Added ur10 and renamed packages
* Contributors: Denis Štogl, IPR-SR2, Kelsey, Mathias Lüdtke, Shaun Edwards, ipa-nhg, jrgnicho, kphawkins, robot
