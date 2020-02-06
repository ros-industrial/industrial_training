# Creating Packages and Nodes
>In this exercise, we will create our own ROS package and node.

## Motivation
The basis of ROS communication is that multiple executables called nodes are running in an environment and communicating with each other in various ways. These nodes exist within a structure called a package. In this module we will create a node inside a newly created package.

## Reference Example
[Create a Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) 

## Further Information and Resources
[Building Packages](http://wiki.ros.org/ROS/Tutorials/BuildingPackages)

[Understanding Nodes](http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)

## Scan-N-Plan Application: Problem Statement
We've installed ROS, created a workspace, and even built a few times. Now we want to create our own package and our own node to do what we want to do.

Your goal is to create your first ROS node:

 1. First you need to create a package inside your catkin workspace.

 2. Then you can write your own node

## Scan-N-Plan Application: Guidance

### Create a Package
1. cd into the catkin workspace src directory
   _Note: Remember that all packages should be created inside a workspace src directory._

   ```
   cd ~/catkin_ws/src
   ```

1. Use the ROS command to create a package called _myworkcell_core_ with a dependency on _roscpp_

   ```
   catkin create pkg myworkcell_core --catkin-deps roscpp
   ```

   See the [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_create.html) documentation for more details on this command.

   * _This command creates a directory and required files for a new ROS package._
   * _The first argument is the name of the new ROS package._
   * _Use `--catkin-deps` to specify packages which the newly created package depends on._

1. There will now be a folder named _myworkcell_core_. Change into that folder and edit the _package.xml_ file. Edit the file and change the description, author, etc., as desired.

   ```
   cd myworkcell_core
   gedit package.xml
   ```

   _If you forget to add a dependency when creating a package, you can add additional dependencies in the _package.xml_ file._



### STOP!  We'll go through a few more lecture slides before continuing this exercise.


### Create a Node
1. In the package folder, create the file _src/vision_node.cpp_ (using _gedit_).

1. Add the ros header (include ros.h).

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>
   ```

1. Add a main function (typical in c++ programs).

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {

   }
   ```

1. Initialize your ROS node (within the main).

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {
     // This must be called before anything else ROS-related
     ros::init(argc, argv, "vision_node");
   }
   ```

1. Create a ROS node handle.

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {
     // This must be called before anything else ROS-related
     ros::init(argc, argv, "vision_node");

     // Create a ROS node handle
     ros::NodeHandle nh;
   }
   ```

1. Print a "Hello World" message using ROS print tools.

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {
     // This must be called before anything else ROS-related
     ros::init(argc, argv, "vision_node");

     // Create a ROS node handle
     ros::NodeHandle nh;

     ROS_INFO("Hello, World!");
   }
   ```

1. Do not exit the program automatically - keep the node alive.

   ``` c++
   /**
   **  Simple ROS Node
   **/
   #include <ros/ros.h>

   int main(int argc, char* argv[])
   {
     // This must be called before anything else ROS-related
     ros::init(argc, argv, "vision_node");

     // Create a ROS node handle
     ros::NodeHandle nh;

     ROS_INFO("Hello, World!");

     // Don't exit the program.
     ros::spin();
   }
   ```

   _ROS_INFO_ is one of the many [logging methods](http://wiki.ros.org/roscpp/Overview/Logging).

   * It will print the message to the terminal output, and send it to the _/rosout_ topic for other nodes to monitor.
   * There are 5 levels of logging: _DEBUG, INFO, WARNING, ERROR, & FATAL._
   * To use a different logging level, replace INFO in _ROS_INFO_ or _ROS_INFO_STREAM_ with the appropriate level.
   * Use _ROS_INFO_ for printf-style logging, and _ROS_INFO_STREAM_ for cout-style logging.

1. Now that you have created the source code for the node, we need to add instructions for building it into an executable program. In the package folder, edit the `CMakeLists.txt` file. Browse through the example rules, and add an executable (`add_executable`) named `vision_node` with the source file named `src/vision_node.cpp`. Also within the `CMakeLists.txt`, make sure your new `vision_node` executable gets linked (`target_link_libraries`) to the catkin libraries.

   ``` diff
   project(myworkcell_core)
   + add_compile_options(-std=c++11)
   
   ...
   
   ###########
   ## BUILD ##
   ###########
   
   include_directories(
     ${catkin_INCLUDE_DIRS}
   )
   
   + add_executable(vision_node src/vision_node.cpp)
   + target_link_libraries(vision_node ${catkin_LIBRARIES})
   ```

   These lines should be added to the `CMakeLists.txt` in the order and location specified above. The commands already exist in the template `CMakeLists.txt` file and can be uncommented and slightly modified  

   * Uncomment existing template examples for `add_compile_options` near the top (just below `project()`)
   * Uncomment and edit existing template examples for `add_executable` and `target_link_libraries` near the bottom in the _BUILD_ section
   * This helps make sure these rules are defined in the correct order, and makes it easy to remember the proper syntax.

   _Note: You're also allowed to spread most of the CMakeLists rules across multiple lines, as shown in the `target_link_libraries` template code_

1. Build your program (node), by running `catkin build` in a terminal window

   * _Remember that you must run `catkin build` from within your `catkin_ws` (or any subdirectory)_
   * This will build all of the programs, libraries, etc. in _myworkcell_core_
   * In this case, it's just a single ROS node _vision_node_

### Run a Node

1. Open a terminal and start the ROS master.

   ```
   roscore
   ```

   _The ROS Master must be running before any ROS nodes can function._

2. Open a second terminal to run your node.

   * In a previous exercise, we added a line to our `.bashrc` to automatically source `devel/setup.bash` in new terminal windows
   * This will automatically export the results of the build into your new terminal session.
   * If you're reusing an existing terminal, you'll need to manually source the setup files (since we added a new node):

     ```
     source ~/catkin_ws/devel/setup.bash
     ```

3. Run your node.

   ```
   rosrun myworkcell_core vision_node
   ```

   _This runs the program we just created. Remember to use TAB to help speed-up typing and reduce errors._

4. In a third terminal, check what nodes are running.

   ```
   rosnode list
   ```

   _In addition to the /rosout node, you should now see a new /vision_node listed._

5. Enter _rosnode kill /vision_node_. This will stop the node.

   _Note: It is more common to use Ctrl+C to stop a running node in the current terminal window._

### Challenge

Goal: Modify the node so that it prints your name. This will require you to run through the build process again.
