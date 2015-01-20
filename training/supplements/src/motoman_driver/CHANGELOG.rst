^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package motoman_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.4 (2014-08-22)
------------------
* Changed message ID for following two message types:
 - ROS_MSG_JOINT_TRAJ_PT_FULL_EX=16 -> ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX=2016
 - ROS_MSG_JOINT_FEEDBACK_EX=17 -> ROS_MSG_MOTO_JOINT_FEEDBACK_EX=2017

1.2.3 (2014-07-02)
------------------
* Corrected ROS_MSG_JOINT_FEEDBACK_EX message.
 - This message will only be sent if the controller has more than one control-group.
 - Single arm systems will not send this message.

1.2.2 (2014-06-26)
------------------
* Corrected the ROS_MSG_MOTO_MOTION_REPLY when replying to
ROS_MSG_JOINT_TRAJ_PT_FULL_EX.  A motion-reply message will be sent for
each control group affected by the multi-group-motion message.  The
motion-reply will correctly indicate the control group index for what it
represents.

1.2.1 (2014-06-16)
------------------
* Primitive I/O support
 - Added custom Motoman-specific message for reading and writing a single I/O point in the controller.
 - Note: Write-support is limited to only certain addresses in the robot controller.  See wiki for details.
* Fixed multiple-arm support for the DX100 controller.

1.2.0 (2014-05-30)
------------------
* Add support for multiple control groups.
 - Support for SDA robots, or multiple individual robots and/or external axes.
 - Add new command message for controlling up to 4 groups.
 - Add new position-feedback message to send all group data.
* Add compatibility for DX200 controller.
* Convert MotoPlusIDE projects into Visual Studio solution.
 - Maintained legacy compatibility for MPIDE.
* Improve I/O feedback signals.
 - Allocate additional signals for future expansion.
 - Add more cases where feedback signals are used.
* Improve error handling
 - Add additional text and I/O feedback in error cases

0.3.3 (2014-02-07)
------------------
* No changes

0.3.2 (2014-01-31)
------------------
* No changes

0.3.1 (2014-01-30)
------------------
* Synchronized versions for bloom release
* driver: move DEPENDS to CATKIN_DEPENDS. Fix `#24 <https://github.com/shaun-edwards/motoman/issues/24>`_.
* driver: link against catkin_LIBRARIES. Fix `#23 <https://github.com/shaun-edwards/motoman/issues/23>`_.
* driver: avoid hardcoded python path. Fix `#19 <https://github.com/shaun-edwards/motoman/issues/19>`_.
* Update move_to_joint.py
* Add proper install targets to driver pkg.
  This fixes `#10 <https://github.com/shaun-edwards/motoman/issues/10>`_.
* Added binaries of motoplus driver.  These can be directly loaded on the controller
* Added controller specific INFORM files
* Commiting motoplus changes required to support DX100 using new incremental motion interface
* Renamed fs100 package to motoman_driver.  The new package now contains drivers for all controllers.  Package name reflects new naming convention
* Contributors: Shaun Edwards, Thomas Timm Andersen, gavanderhoorn
