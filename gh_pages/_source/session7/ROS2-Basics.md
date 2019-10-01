# ROS2 Basics Exercise

## Motivation
Our goal for this exercise is to go through the basic ROS2 commands to understand how they work. We will not be writing any code during this exercise, instead we will just run through a few commands in the shell.

1. Start by creating a new ROS2 workspace. This will be similar to ROS1's catkin in that it consists of a folder with a src/ subdirectory. However, there is no such thing as `colcon init`.

1. You will need to source /opt/ros/dashing/setup.bash instead of melodic, check your .bashrc!

1. Clone the ROS2 demos repository which we will use to run some examples. Make sure you're on the dashing branch!
   ```bash
   cd src/
   git clone git@github.com:ros2/demos.git
   git clone git@github.com:ros2/examples.git
   ```

1. Run rosdep to install all necessary dependencies
   ```bash
   rosdep install -i --from-paths src
   ```
1. Build with colcon: `colcon build`. Keep in mind that this will create colcon build, install, and log folders where it's run so you will likely want to run this from the workspace root directory!

1. Inspect build packages
   ```
   ros2 pkg list
   ros2 pkg executables
   ```

1. Run a node (no roscore needed!). For examples:
   ```
   ros2 run <package_name> <executable_file>
   ros2 run demo_nodes_cpp listener
   ros2 run demo_nodes_cpp talker
   ```

1. Inspecting topics and messages
   ```
   ros2 node list
   ros2 node info /talker
   ros2 topic list
   ros2 topic info /chatter
   ros2 echo /chatter
   ros2 msg packages
   ros2 msg list
   ```

1. The general syntax for running a launch file is - `ros2 launch <package_name> <launch_file>`
   ```
   ros2 launch dummy_robot_bringup dummy_robot_bringup.launch.py
   ros2 run rviz2
   ```