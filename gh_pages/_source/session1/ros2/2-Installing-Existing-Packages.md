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
   ros2 pkg prefix nav_2d_msgs
   ```

   * If successful, this command prints the directory where ROS `nav_2d_msgs` package is installed.
   * You should see an error message *Package not found*.
   * _This package is not installed on the system, so we will install it._


1. Use the _APT_ package manager to try to install the package.

   ```
   apt install ros-eloquent-nav-2d-msgs
   ```

   * Note that dashes are used for the APT package name even though the ROS name uses underscores.
   * The program will say it cannot install the package, and suggests that we must run the program as root.
   * Try pressing the _TAB_ key while typing the package name.
     * The system will try to automatically complete the package name, if possible.
     * Frequent use of the TAB key will help speed up entry of many typed commands.


1. Install using _sudo_.

   ```
   sudo apt install ros-eloquent-nav-2d-msgs
   ```

   * Note the use of the _sudo_ command to run a command with "root" (administrator) privileges.
   * Enter your password, and (if asked) confirm you wish to install the program.


1. Search for the package again.

   ```
   ros2 pkg prefix nav_2d_msgs
   ```

   * This time, you will see a directory output of _/opt/ros/eloquent_.


1. Remove the package from the system.

   ```
   sudo apt remove ros-eloquent-nav-2d-msgs
   ```

   * _Don't worry. We won't be needing this package for any future exercises, so it's safe to remove._


### Download and Build a Package from Source

1. Identify the source repository for the desired package:
   1. Go to [github](http://github.com/search).
   1. Search for fake_ar_publisher.
   1. Click on this repository, and look to the right for the _Clone or Download_, then copy to clipboard.

1. Clone the _fake_ar_publisher_ [repository](https://github.com/ros-industrial/fake_ar_publisher.git) into the workspace's _src_ directory.

   ```
   cd ~/ros2_ws/src
   git clone -b ros2 https://github.com/ros-industrial/fake_ar_publisher.git
   ```

   * _Use Ctrl-Shift-V to paste within the terminal, or use your mouse to right-click and select paste_
   * _Git commands are outside of the scope of this class, but there are good tutorials available [here](https://help.github.com/articles/git-and-github-learning-resources/)_
   * _Specifying the correct branch name with the `-b` is important since repositories often contain multiple incompatible versions on different branches_.

1. Build the new package using `colcon build` inside _~/ros2_ws/_

1. Once the build completes, the contents of the workspace have changed and the setup.bash file must be re-sourced in order to see the new package.

   * In the previous exercise, we added a line to our `~/.bashrc` file to automatically re-source the setup files in each new terminal.
   * This is sufficient for most development activities, but you may sometimes need to re-execute the `source` command in your current terminal (e.g. when adding new packages):

     ```
     source ~/ros2_ws/install/setup.bash
     ```

1. Once the build completes, explore the _build_ and _install_ directories to see what files were created.

1. Use _ros2 pkg_ to verify the new packages are visible to ROS.

   ```
   ros2 pkg prefix fake_ar_publisher
   ```
 
   * This is a helpful command to troubleshoot problems with a ROS workspace.
   * If ROS can't find your package, try re-building the workspace and then re-sourcing the workspace's `setup.bash` file.
