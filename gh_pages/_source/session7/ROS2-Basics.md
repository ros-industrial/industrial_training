# ROS2 Basics Exercise

## Motivation

Our goal for this exercise is to go through the basic ROS2 commands to understand how they work. We
will not be writing any code during this exercise, instead we will just run through a few commands
in the shell.

## Workspaces and packages

1.  Start by creating a new ROS2 workspace. As with catkin in ROS1, a workspace consists of a folder
    with a `src/` subdirectory.

    ```bash
    mkdir -p ~/ros2_ws/src
    ```

1.  Clone the ROS2 demos examples repositories which we will use to run some examples. When building
    existing packages from source, it is important to verify the code version as the most recent
    version may have changes incompatible with the current release. Here we checkout the
    repositories to the `eloquent` release branch when cloned.

    ```bash
    cd ~/ros2_ws/src
    git clone -b eloquent git@github.com:ros2/demos.git
    git clone -b eloquent git@github.com:ros2/examples.git
    ```

1.  Install any required dependencies for the repositories using rosdep. This step is the same as in
    ROS1 since rosdep is installed as a system tool.

    ```bash
    cd ~/ros2_ws
    rosdep install --ignore-src --from-paths src/
    ```

## Building packages

ROS2 uses `colcon` as the build tool for ROS packages. Colcon is installed separately from ROS
distributions as a pure Python package. It is always available in the user's PATH without sourcing a
workspace setup file.

1.  Run `colcon -h` to see a short help summary and a list of verbs that can be used. Run `colcon
    build -h` to see the help description for the `build` verb.

1.  Run `colcon list` and `colcon info` to see infomation about what packages are currently in the
    workspace. Colcon can be directed to ignore packages if a directory contains an empty file named
    `COLCON_IGNORE`.

1.  Build the workspace. This will require a setup script to be sourced so Ament can find the
    required dependent ROS packages. Check your `.bashrc` file and either change or remove any line
    that sources a ROS1 setup file at the end of the file. Remember if you change your `.bashrc` to
    start a terminal for it to take effect.

    ```bash
    source /opt/ros/eloquent/setup.bash
    colcon build
    ```

    - **Important**: Unlike catkin tools, colcon looks for packages and will create its output folders
    wherever you run it. Be sure you are in the workspace root before running it.

    - Colcon does not currently have a `clean` verb. To rebuild a workspace from scratch, you must
    remove the generated outputs manually: `rm -r build/ install/ log/`

1.  Source the newly-built workspace. Inside the new `install/` folder will be both `setup.bash` and
    `local_setup.bash` files. The `setup.bash` will configure a terminal environment to see the
    workspace packages as well as the environment the workspace was built in. `local_setup.bash`
    will add the workspace packages to the current environment.

    - If you keep your `.bashrc` to source `/opt/ros/eloquent/setup.bash` then you should only need to
    run `source install/local_setup.bash`

    - Otherwise, run `source install/setup.bash`

## The `ros2` command

ROS2 replaces the set of `ros*` command line tools with subcommands of a single `ros2` command.
Briefly we'll run through some of ones that may be useful for a developer.

### `ros2 pkg`

Shows information about ROS packages visible in the current environment

- Use `ros2 pkg list` to see a list of all visible packages

- Use `ros2 pkg executables` to see a list of executables made available by a package. E.g., `ros2
pkg executables tf2_ros`

- Use `ros2 pkg prefix` to get the installation location of package

### `ros2 run`

Runs executables provided by a package (`ros2 run <package_name> <executable_file>`)

- Open two terminals sourced to the `ros2_ws` workspace.

- In the first one, run `ros2 run demo_nodes_cpp listener`. Tab completion should be available so
you don't have to type the full package and executable names.

- In the second, run `ros2 run demo_nodes_cpp talker`. You
should now start to see a string message start to be printed repeatedly on both terminals. Leave
both nodes running.

*Note*: You did not need to start a ROS master process before running nodes. The two nodes
discovered each other on the network in a decentralized way as they started up.

### `ros2 node`

Shows information about currently running ROS nodes.

- Open a third terminal sourced to the `ros_ws` workspace.

- Run `ros2 node list` to see a list of running nodes

- Run `ros2 node info /talker` to see the topics and services the talker node is using

### `ros2 topic`

Shows information about topics currently used by one or more nodes.

- Run `ros2 topic list` to see currently published topics

- Run `ros2 topic info /chatter` to see information about the topic the talker node is publishing

- Run `ros2 topic echo /chatter` to locally subscribe to the `/chatter` topic and print it on the
terminal

- Run `ros2 topic -h` for a list of more available subcommands

### `ros2 service`/`srv`

Shows information about ROS services.

- Either stop the talker and listener nodes with `Ctrl-C` or open two more terminals.

- In the first terminal, run `ros2 run demo_nodes_cpp add_two_ints_server`

- In the second terminal, run `ros2 service list -t` to show currently available services and what
type they are.

- Run `ros2 srv show example_interfaces/srv/AddTwoInts` to see the service definition of the
`/add_two_ints` service. Notice the names and types of the fields used in the service request.

- Run `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts '{a: 1, b: 2}'` to manually
call the service. The fields of the service request must be set using a YAML dictionary that
specifies the field names and values.

### `ros2 launch`

Start an installed launch file (`ros2 launch <package_name> <launch_file>`)

- Close any currently running nodes

- Run `ros2 launch dummy_robot_bringup dummy_robot_bringup.launch.py`. Again, tab completion should be
able to help here. Notice the `.py` suffix on the launch file, it's just a python script.

This starts five nodes that can inspected with the above command line tools. You might notice a
`/joint_states` topic publishing changing values and a `/scan` topic publishing simulated 2D laser
scanner data. Let's visualize them.

## RViz

RViz has been renamed to `rviz2` and substantially rewritten for ROS2 but appears more-or-less the
same from the user's point-of-view.

- In a free terminal, run `rviz2` while the dummy robot launch file is still running.

- Change the 'Fixed Frame' to 'world'

- Add a `RobotModel` display type. Expand the display options and set 'Description Source' to
'Topic' and 'Description Topic' to '/robot_description'. You should now see a robot moving on your
screen.

- Add a `LaserScan` display and have it display the '/scan' topic. You should start to see 2D laser
scan data appear as if the scanner were attached to the end of the robot. Getting intermittent
transform errors is normal.
