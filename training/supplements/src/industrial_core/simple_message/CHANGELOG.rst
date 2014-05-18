^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package simple_message
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
