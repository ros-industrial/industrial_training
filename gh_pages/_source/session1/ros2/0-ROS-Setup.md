# ROS Setup
> In this exercise, we will setup ROS to be used from the terminal.
***

## Motivation
In order to start programming in ROS, you should know how to install ROS on a new machine as well and check that the installation worked properly. This module will walk you through a few simple checks of your installed ROS system. Assuming you are working from the VM, you can skip any installation instructions as ROS is already installed.

## Reference Example
[Configuring ROS](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/)

## Further Information and Resources
[ROS2 Eloquent Installation Instructions](https://index.ros.org/doc/ros2/Installation/Eloquent/)

## Scan-N-Plan Application: Problem Statement
We believe we have a good installation of ROS but let's test it to make sure.

## Scan-N-Plan Application: Guidance
### Setup ~/.bashrc
1. If you are ever having problems finding or using your ROS packages make sure that you have your environment properly setup. A good way to check is to ensure that environment variables that ROS sets are present:

   ```
   printenv | grep ROS
   printenv | grep AMENT
   ```

1. If they are not then you might need to 'source' some setup.*sh files.

   ```
   source /opt/ros/eloquent/setup.bash
   ```

1. In a "bare" ROS install, you will need to run this command on every new shell you open to have access to the ROS commands.  One of the setup steps in a _typical_ ROS install is to add that command to the end of your `~/.bashrc` file, which is run automatically in every new terminal window.  Check that your `.bashrc` file has already been configured to source the ROS-melodic `setup.bash` script:

   ```
   tail ~/.bashrc
   ```

This process allows you to install several ROS distributions (even ROS1 and ROS2) on the same computer and switch between them by sourcing the distribution-specific `setup.bash` file.
