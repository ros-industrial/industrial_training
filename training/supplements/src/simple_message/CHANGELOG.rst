^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_message
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.4 (2014-01-21)
------------------
* robot_client: workaround for `#46 <https://github.com/shaun-edwards/industrial_core/issues/46>`_. Fix `#67 <https://github.com/shaun-edwards/industrial_core/issues/67>`_.
  This is an updated version of the workaround committed in 9df46977. Instead
  of requiring dependent packages to invoke the function defined in the
  CFG_EXTRAS cmake snippet, the snippet now sets up the linker path directly.
  Dependent packages now only need to remember to explicitly list their
  dependency on `industrial_robot_client` and `simple_message` in their
  `add_library(..)` statements.
* Contributors: gavanderhoorn

0.3.3 (2014-01-13)
------------------
* Fixed build issue due simple message library linking
* Contributors: gavanderhoorn

0.3.2 (2014-01-10)
------------------
* Removed header from industrial_utils/utils.h (not required)

0.3.1 (2014-01-09)
------------------
* Added polling check to socket read and muiltiple read calls in order to receive all desired bytes
* Removed library export from catkin macro.  Packages that depend on these must declare library dependencies explicitly (by name)
* Add error message to socket errors (instead of just errno).
* Converted to catkin
* Contributors: Christina Gomez, JeremyZoss, ROS, Shaun Edwards, gavanderhoorn, jrgnicho, kphawkins, shaun-edwards
