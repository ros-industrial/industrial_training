^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package industrial_robot_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.4 (2014-01-21)
------------------
* No change

0.3.3 (2014-01-13)
------------------
* No change

0.3.2 (2014-01-10)
------------------
* Removed header from industrial_utils/utils.h (not required)

0.3.1 (2014-01-09)
------------------
* robot_simulator: avoid hardcoded python path. Fix `#53 <https://github.com/shaun-edwards/industrial_core/issues/53>`_.
* Remove obsolete export tags. Fix `#43 <https://github.com/shaun-edwards/industrial_core/issues/43>`_.
* Add install target for launchfile in sim pkg.
  Fix `#35 <https://github.com/shaun-edwards/industrial_core/issues/35>`_.
* Fix issue `#6 <https://github.com/shaun-edwards/industrial_core/issues/6>`_ (Install target fails for industrial_robot_simulator): setup.py file not required for python executables, see http://ros.org/wiki/catkin/migrating_from_rosbuild
* bugFix: `#61 <https://github.com/shaun-edwards/industrial_core/issues/61>`_ - fix joint-name remapping in industrial_robot_simulator
* Added interpolated motion to MotionControllerSimulator class. Includes addition of interpolate(), and modification of  _motion_worker()
* New rospy.get_param() added to IndustrialRobotSimulatorNode in order to assign motion_update_rate
* Converted to catkin
* Contributors: JeremyZoss, Shaun Edwards, dpsolomon, gavanderhoorn, jrgnicho
