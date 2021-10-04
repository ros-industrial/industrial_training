# ROS Setup
> In this exercise, we will setup ROS to be used from the terminal.
***

## Motivation
In order to start programming in ROS, you should know how to install ROS on a new machine as well and check that the installation worked properly. This module will walk you through a few simple checks of your installed ROS system. Assuming you are working from the VM, you can skip any installation instructions as ROS is already installed.

## Reference Example
[Configuring ROS](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/)

## Further Information and Resources
[ROS2 Foxy Installation Instructions](https://index.ros.org/doc/ros2/Installation/Foxy/)

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
   source /opt/ros/foxy/setup.bash
   ```

1. Now repeat the check from above and verify that the environment variables are now present.

1. This process allows you to install several ROS distributions (even ROS1 and ROS2) side-by-side on the same computer and switch between them by sourcing the distribution-specific `setup.bash` file.  Your training PC also has a ROS1 "noetic" distribution installed.  Try activating that distribution in the same terminal window you used above and observe how the environment variables change.

   ```
   source /opt/ros/noetic/setup.bash
   env | grep ROS
   ```

1. If you switch back to the "foxy" ROS2 distribution, you'll notice that your environment is stil polluted with some variables from the ROS1 "noetic" distribution.  To prevent confusion, it can be helpful to always start with a fresh terminal window rather than switching between distributions in the same terminal window.

In a "bare" ROS install, you will need to run the "source" command on **every** new shell you open to have access to the ROS commands.  If your project only uses a single distribution, it can be helpful to configure our `~/.bashrc` file to automatically source this setup file for each new terminal window.  See [here](https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html#add-sourcing-to-your-shell-startup-script) for details.  Since this class uses content in both ROS1 and ROS2, we have not configured this auto-sourcing behavior for your training PC.

1. Open a new terminal window and source the ROS2 "foxy" distribution.  Experiment with terminal windows, sourcing, and checking your environment until you are comfortable with this concept.

**Remember to source the appropriate `setup` script in every new terminal window you open.  Forgetting this step is a common error among new ROS users.**
