# ROS Setup
> In this exercise, we will setup ROS to be used from the terminal.
***

## Motivation
In order to start programming in ROS, you should know how to install ROS on a new machine as well and check that the installation worked properly. This module will walk you through a few simple checks of your installed ROS system. Assuming you are working from the VM, you can skip any installation instructions as ROS is already installed.

## Reference Example
[Configuring ROS](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)

## Further Information and Resources
[ROS2 Humble Installation Instructions](https://docs.ros.org/en/humble/Installation.html)

## Scan-N-Plan Application: Problem Statement
We believe we have a good installation of ROS but let's test it to make sure.

## Scan-N-Plan Application: Guidance
### Check Environment
1. If you are ever having problems finding or using your ROS packages make sure that you have your environment properly setup. A good way to check is to ensure that environment variables that ROS sets are present:

   ```
   env | grep ROS
   env | grep AMENT
   ```

1. If they are not then you might need to 'source' some setup.*sh files.

   ```
   source /opt/ros/humble/setup.bash
   ```

1. Now repeat the check from above and verify that the environment variables are now present.

In a "bare" ROS install, you will need to run the "source" command on **every** new shell you open to have access to the ROS commands.  If your project only uses a single distribution, it can be helpful to configure our `~/.bashrc` file to automatically source this setup file for each new terminal window.  See [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#add-sourcing-to-your-shell-startup-script) for details. If you will be using multiple ROS distributions on one machine, then you do NOT want this auto-sourcing behavior. The auto-sourcing will always source the same distribution, and sourcing another distribution after this can cause issues. This will not be an issue in this class as we will exclusively using ROS 2 Humble.

1. Open a new terminal window and source the ROS2 "humble" distribution.  Experiment with terminal windows, sourcing, and checking your environment until you are comfortable with this concept.

**Remember to source the appropriate `setup` script in every new terminal window you open.  Forgetting this step is a common error among new ROS users.**
