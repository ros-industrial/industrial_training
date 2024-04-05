# Create ROS Workspace
> In this exercise, we will create and build an empty ROS workspace.

## Motivation
Any ROS project begins with making a workspace. In this workspace, you will put all the things related to this particular project. In this module we will create the workspace where we will build the components of our Scan-N-Plan application.

## Reference Example
Steps to creating a workspace: [Creating a Workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

## Further Information and Resources
Additional details on what a ROS environment consists of and some basic checks you can run: [Configuring a ROS2 Environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)

## Scan-N-Plan Application: Problem Statement
We have a good installation of ROS, and we need to take the first step to setting up our particular application. Your goal is to create a workspace for your application and its supplements.

## Scan-N-Plan Application: Guidance

### Create a Workspace

1. Close any open terminal windows.  Open a new terminal and source the "humble" distribution.

   ```
   source /opt/ros/humble/setup.bash
   ```

1. Create the root workspace directory. Note that the required structure is a top-level directory and a `src/` directory one level down. The directory name is a completely free choice. We will use `ros2_ws` throughout these tutorials.

   ```
   cd ~/
   mkdir --parents ros2_ws/src
   cd ros2_ws
   ls
   ```

1. Build the workspace from the workspace root-directory (`ros2_ws`).

   ```
   colcon build
   ls
   ```

   * _See that the `ros2_ws` directory now contains additional directories (build, install, log)._
   * `colcon build` must always be run from the **root** of your workspace directory.
   * Don't run `colcon build` from a terminal where you've also sourced this workspace's setup file.  Use a dedicated terminal for building.
   
1. These new directories can be safely deleted at any time.  This is sometimes used to resolve unusual build errors by "starting with a clean slate".  Note that colcon never changes any files in the `src` directory.  Re-run `colcon build` to re-create the build/install/log directories.

   ```
   rm -r build/ install/ log/
   ls
   colcon build
   ls
   ```

1. Make the contents of this workspace visible to ROS. Source the setup file in the install directory.

   ```
   source ~/ros2_ws/install/setup.bash
   ```

   * _This file MUST be sourced for every new terminal._
   * This file will automatically include the ROS environment that was active when you first built this workspace.  Make sure you have sourced the correct ROS distribution before building a new workspace for the first time!


