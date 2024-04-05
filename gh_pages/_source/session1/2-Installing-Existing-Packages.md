# Installing Packages

## Motivation
Many of the coolest and most useful capabilities of ROS already exist somewhere in its community. Often, stable resources exist as easily downloadable debian packages. Alternately, some resources are less tested or more "cutting edge" and have not reached a stable release state; you can still access many of these resources by downloading them from their repository (usually housed on Github). Getting these git packages takes a few more steps than the debian packages. In this module we will access both types of packages and install them on our system.

## Reference Example
[apt-get usage](http://www.tecmint.com/useful-basic-commands-of-apt-get-and-apt-cache-for-package-management/)

## Further Information and Resources
[Ubuntu apt-get How To](https://help.ubuntu.com/community/AptGet/Howto)

[Git Get Repo](https://git-scm.com/book/en/v2/Git-Basics-Getting-a-Git-Repository)

[Git Clone Documentation](https://git-scm.com/docs/git-clone)

## Scan-N-Plan Application: Problem Statement
We have a good installation of ROS, and we have an idea of some packages that exist in ROS that we would like to use within our program. We have found a package which is stable and has a debian package we can download. We've also found a less stable git package that we are interested in. Go out into the ROS world and download these packages!

1. A certain message type exists which you want to use. The stable ROS package is called: nav_2d_msgs

1. You are using an AR tag, but for testing purposes you would like a node to publish similar info : fake_ar_publisher

Your goal is to have access to both of these packages' resources within your package/workspace:

1. nav_2d_msgs (using apt-get)

1. fake_ar_publisher (from git)

## Scan-N-Plan Application: Guidance

### Install Package from apt Repository

1. Open a terminal window. Use the `ros2 pkg` command to search for a package.

   ```
   ros2 pkg --help
   ros2 pkg prefix nav_2d_msgs
   ```

   * If successful, this command prints the directory where ROS `nav_2d_msgs` package is installed.
   * You should see an error message *Package not found*.
   * _This package is not installed on the system, so we will install it._


1. Use the _APT_ package manager to try to install the package.

   ```
   apt install ros-humble-nav-2d-msgs
   ```

   * Note the naming convention for the APT package name:
     * ROS APT packages follow this naming pattern: `ros-<distro>-<package>`.
     * underscores ("_") in the package name are replaced with dashes ("-").
   * The program will say it cannot install the package, and suggests that we must run the program as root.
   * Try pressing the _TAB_ key while typing the package name.
     * The system will try to automatically complete the package name, if possible.
     * Frequent use of the TAB key will help speed up entry of many typed commands and provide early detection of typing errors or environment-setup issues.


1. Install using _sudo_.

   ```
   sudo apt install ros-humble-nav-2d-msgs
   ```

   * Note the use of the _sudo_ command to run a command with "root" (administrator) privileges.
   * Enter your password, and (if asked) confirm you wish to install the program.


1. Search for the package again.

   ```
   ros2 pkg prefix nav_2d_msgs
   ```

   * Linux Tip: use the up/down arrow keys to scroll through previous command history, to avoid re-typing common commands.
   * This time, you will see a directory output of _/opt/ros/humble_.


1. Remove the package from the system.

   ```
   sudo apt remove ros-humble-nav-2d-msgs
   ```

   * _Don't worry. We won't be needing this package for any future exercises, so it's safe to remove._


### Download and Build a Package from Source

1. Identify the source repository for the desired package:
   1. Go to [github](http://github.com/search).
   1. Search for fake_ar_publisher.
   1. Click on this repository, and look to the right for the _Code_ link, then copy the URL listed under _Clone_ to the clipboard.

1. Open a new terminal and clone the _fake_ar_publisher_ [repository](https://github.com/ros-industrial/fake_ar_publisher.git) into the workspace's _src_ directory.

   ```
   cd ~/ros2_ws/src
   git clone -b ros2 https://github.com/ros-industrial/fake_ar_publisher.git
   ```

   * _Use Ctrl-Shift-V to paste within the terminal, or use your mouse to right-click and select paste_
   * _Git commands are outside of the scope of this class, but there are good tutorials available [here](https://help.github.com/articles/git-and-github-learning-resources/)_
   * _Specifying the correct branch name with the `-b` is important since repositories often contain multiple incompatible versions on different branches_.

1. Use `ros2 pkg prefix` to see if this package is visible from ROS:

   ```
   ros2 pkg prefix fake_ar_publisher
   ```
   
   * You may see an error ("ros2: command not found").  Remember that none of the ROS commands are visible until you've sourced the base ROS distro `setup.bash` file (either directly, or chained through your development workspace's `setup.bash`).
   
   * Even after you've sourced the appropriate `setup.bash` file, the package is still not visible.  Packages you download as source must first be built before they're visible to ROS.

1. Switch back to your "build terminal" and build the new package using `colcon build`

   * remember this command must be run from your workspace root (_~/ros2_ws/_)

1. Once the build completes, explore the _build_ and _install_ directories to see what files were created.

1. Switch back to your 2nd terminal and again use `ros2 pkg prefix` to see if ROS can find the package.

   * This is a helpful command to troubleshoot problems with a ROS workspace.  If this command can't find your package, other nodes can't either.
   * The package is still not visible to ROS.  Even if you've previously sourced the `setup.bash` file in this terminal, you must re-source the setup file after adding new packages to the terminal.  You don't need to re-source the file after small changes (like recompiling code edits).

1. Re-run the source command to make the new package visible to ROS, then verify that ROS can see it using the `ros2 pkg prefix` command:

   ```
   source ~/ros2_ws/install/setup.bash
   ros2 pkg prefix fake_ar_publisher
   ```
   
   * remember to use TAB while typing.  This can be an even quicker test than using the `ros2 pkg prefix` command.

