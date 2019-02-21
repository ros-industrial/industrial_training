# ROS-Setup
> In this exercise, we will setup ROS to be used from the terminal, and start roscore
***

## Motivation
In order to start programming in ROS, you should know how to install ROS on a new machine as well and check that the installation worked properly. This module will walk you through a few simple checks of your installed ROS system. Assuming you are working from the VM, you can skip any installation instructions as ROS is already installed.

## Reference Example
[Configuring ROS](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

## Further Information and Resources
[Installation Instructions](http://wiki.ros.org/melodic/Installation/Ubuntu)

[Navigating ROS](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem)

## Scan-N-Plan Application: Problem Statement
We believe we have a good installation of ROS but let's test it to make sure.

## Scan-N-Plan Application: Guidance
### Setup ~/.bashrc
1. If you are ever having problems finding or using your ROS packages make sure that you have your environment properly setup. A good way to check is to ensure that environment variables like ROS_ROOT and ROS_PACKAGE_PATH are set:

   ```
   printenv | grep ROS
   ```

2. If they are not then you might need to 'source' some setup.*sh files.

   ```
   source /opt/ros/melodic/setup.bash
   ```

3. In a "bare" ROS install, you will need to run this command on every new shell you open to have access to the ROS commands.  One of the setup steps in a _typical_ ROS install is to add that command to the end of your `~/.bashrc` file, which is run automatically in every new terminal window.  Check that your `.bashrc` file has already been configured to source the ROS-melodic `setup.bash` script:

   ```
   tail ~/.bashrc
   ```

This process allows you to install several ROS distributions (e.g. indigo, kinetic, melodic) on the same computer and switch between them by sourcing the distribution-specific `setup.bash` file.

### Starting roscore
1. _roscore_ is a collection of nodes and programs that are pre-requisites of a ROS-based system. You must have a roscore running in order for ROS nodes to communicate. It is launched using the _roscore_ command.

   ```
   roscore
   ```

   _roscore_ will start up:

   * a ROS Master
   * a ROS Parameter Server
   * a rosout logging node

   You will see ending with `started core service [/rosout]`. If you see `roscore: command not found` then you have not sourced your environment, please refer to section 5.1. .bashrc Setup.

2. To view the logging node, open a new terminal and enter:

   ```
   rosnode list
   ```
 
   The logging node is named _/rosout_

3. Press _Ctrl+C_ in the first terminal window to stop roscore.  Ctrl-C is the typical method used to stop most ROS commands.
