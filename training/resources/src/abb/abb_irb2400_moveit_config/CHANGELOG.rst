^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package abb_irb2400_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.4 (2014-12-14)
------------------
* Merged release artifacts from hydro branch
* irb2400_moveit_cfg: add missing run_depend on ikfast plugin. Fix `#53 <https://github.com/ros-industrial/abb/issues/53>`_.
* Contributors: Shaun Edwards, gavanderhoorn

1.1.3 (2014-09-05)
------------------
* Bump versions.
* irb2400: update MoveIt config to use IKFast plugin.
* irb2400: (re)add plan execution support to MoveIt package.
* irb2400: add basic (regenerated) MoveIt package.
  Regenerated package, updated version of the irb_2400_moveit_config.
  This uses the files from the abb_irb2400_support package.
  Note: acceleration limits are identical to those specified in the
  old irb_2400_moveit_config package (ie: 1.0 m/s^2), which should
  be updated to more realistic values.
* Contributors: gavanderhoorn
