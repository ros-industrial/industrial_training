# Creating Packages and Nodes
>In this exercise, we will create our own ROS package and node.

## Motivation
The basis of ROS communication is that multiple executables called nodes are running in an environment and communicating with each other in various ways. These nodes exist within a structure called a package. In this module we will create a node inside a newly created package.

## Reference Example
[Create a Package](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package)

## Further Information and Resources
* [Package.xml Format](https://ros.org/reps/rep-0149.html#data-representation)
* [Understanding Nodes](https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Nodes)

## Scan-N-Plan Application: Problem Statement
We've installed ROS, created a workspace, and even built a few times. Now we want to create our own package and our own node to do what we want to do.

Your goal is to create your first ROS node:

 1. First you need to create a package inside your workspace.

 2. Then you can write your own node

## Scan-N-Plan Application: Guidance

### Create a Package
1. cd into the workspace src directory
   * _Remember that all packages should be created inside a workspace src directory._

   ```
   cd ~/ros2_ws/src
   ```

1. Use the ROS command to create a package called _myworkcell_core_ with a dependency on _rclcpp_

   ```
   ros2 pkg create myworkcell_core --dependencies rclcpp
   ```

   Run `ros2 pkg create -h` to see more details and options for this command.

   * _This command creates a directory and required files for a new ROS package._
   * _The first argument is the name of the new ROS package._
   * _Use the `--dependencies` option to specify packages which the newly created package depends on._

1. There will now be a folder named _myworkcell_core_. Change into that folder and edit the _package.xml_ file. Edit the file and change the description, author, etc., as desired.

   ```
   cd myworkcell_core
   gedit package.xml &
   ```

   _If you forget to add a dependency when creating a package, you can add additional dependencies in the _package.xml_ file._


### STOP!  We'll go through a few more lecture slides before continuing this exercise.


### Create a Node
1. In the package folder, create the file _src/vision_node.cpp_ (using _gedit_).
  * Make sure you know the difference between the _workspace_ `src` directory and the myworkcell_core _package_ `src` directory.  This node source-file should live at: `~/ros2_ws/src/myworkcell_core/src/vision_node.cpp`.

1. Add the ROS C++ header (include rclcpp.hpp).

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <rclcpp/rclcpp.hpp>
   ```

1. Add a main function (typical in c++ programs). The rest of our code will go inside this function.

   ``` c++

   int main(int argc, char* argv[])
   {

   }
   ```

1. Initialize the ROS CPP middleware interface (must be called once per process).

   ``` c++
   // This must be called before anything else ROS-related
   rclcpp::init(argc, argv);
   ```

1. Create a ROS node instance.

   ``` c++
   auto node = std::make_shared<rclcpp::Node>("vision_node");
   ```

1. Print a "Hello World" message using ROS print tools.

   ``` c++
   RCLCPP_INFO(node->get_logger(), "Hello, World!");
   ```

1. Do not exit the program automatically - keep the node alive.

   ``` c++
   // Don't exit the program.
   rclcpp::spin(node);
   ```

   The complete code should look like this:

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <rclcpp/rclcpp.hpp>

   int main(int argc, char* argv[])
   {
     // This must be called before anything else ROS-related
     rclcpp::init(argc, argv);

     // Create a ROS node
     auto node = std::make_shared<rclcpp::Node>("vision_node");

     RCLCPP_INFO(node->get_logger(), "Hello, World!");

     // Don't exit the program.
     rclcpp::spin(node);
   }
   ```

   `RCLCPP_INFO` is one of the many [logging methods](http://docs.ros2.org/foxy/api/rclcpp/logging_8hpp.html)

   * It will print the message to the terminal output, and send it to the _/rosout_ topic for other nodes to monitor.
   * There are 5 levels of logging: _DEBUG, INFO, WARNING, ERROR, & FATAL._
   * To use a different logging level, replace INFO in `RCLCPP_INFO` or `RCLCPP_INFO_STREAM` with the appropriate level.
   * Use `RCLCPP_INFO` for printf-style logging, and `RCLCPP_INFO_STREAM` for cout-style logging.

   `rclcpp::spin(node)` instructs the node to listen for any activity on subscribed topics or requested service connections. Without it, your callback functions won't ever run. Don't worry if some of this language doesn't yet make sense, we'll cover topics and services in the next sections. (More technical details [here](https://docs.ros.org/en/rolling/Concepts/About-Executors.html).)

1. Now that you have created the source code for the node, we need to add instructions for building it into an executable program. In the package folder, edit the _CMakeLists.txt_ file using _gedit_. Define an executable node named vision_node (_add_executable_), with the source file _vision_node.cpp_. Also within the _CMakeLists.txt_, make sure your new vision_node gets linked to the required dependencies.

   ``` cmake
   add_executable(vision_node src/vision_node.cpp)
   ament_target_dependencies(vision_node rclcpp)
   ```

   These lines should be place after the section labeled `# find dependencies` and before the `ament_package()` call. The flow of a _CMakeLists.txt_ file is almost always to first find all dependencies the current project requires, then to declare the _targets_ the current project is creating, and finally to specify extra information such as how to test the targets and things that projects making use of this one need to know. Note that the term "project" is used here in the context of CMake. Each ROS package acts as a separate CMake project (notice the `project(myworkcell_core)`) line at the very top of the file). 

  * See [ament_cmake documentation](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html) for more details on common ROS2 CMakeLists rules.
  * _You're allowed to spread most of the CMakeLists rules across multiple lines, which is often required when a target contains many source files or has many dependencies._

1. We've now told CMake about the _vision_node_ executable and how to build it, but to actually run it, the file must be *installed* along with all the other workspace outputs. Typically, the installation location will be the `install/` directory alongside the `src/` directory. Add the following lines after `ament_target_dependencies` but before `ament_package` into your CMakeLists.txt to declare an installation rule for the _vision_node_ executable:

   ``` cmake
   # Mark executables for installation
   install(TARGETS vision_node
       DESTINATION lib/${PROJECT_NAME})
   ```
   
   * _ROS2 installs executables into a package-named subdir in the `lib` directory._
   * _A different set of `install` rules are needed if your project builds libraries, which are installed in bot the root `lib` and `bin` directories._

1. Build your program (node), by running `colcon build` in your build terminal window

   * _Remember that you must run `colcon build` from the `ros2_ws` directory._
   * This will build all of the programs, libraries, etc. in _myworkcell_core_
   * In this case, it's just a single ROS node _vision_node_

### Run a Node

1. Open a terminal to run your node.

   * Remember to run `source ~/ros2_ws/install/setup.bash` in this new terminal

1. Run your node.

   ```
   ros2 run myworkcell_core vision_node
   ```

   _This runs the program we just created. Remember to use TAB to help speed-up typing and reduce errors._

1. In a second terminal, check what nodes are running.

   ```
   ros2 node list
   ```

   _You should now see a new /vision_node listed._

1. To stop the node, input Ctrl+C in the terminal where it is running. The node is running as a completely normal executable file so standard Linux process management tools can also stop it (e.g., `killall vision_node`).

### Challenge

Goal: Modify the node so that it prints your name. This will require you to run through the build process again.
