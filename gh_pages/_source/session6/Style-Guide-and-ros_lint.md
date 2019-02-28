# ROS Style Guide and ros_lint

## Motivation 
The ROS Scan-N-Plan application from Exercise 4.0 is complete, tested and documented.  Now we want to clean up the code according to the style guide so other developers can easily understand our work.

## Information and Resources 

[The Official ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)

[Automated Style Guide Enforcement](http://wiki.ros.org/roslint)

## Scan-N-Plan Application: Problem Statement
We have completed and tested our Scan-N-Plan program from Exercise 4.0 and we need to release the code to the public. Your goal is to ensure the code we have created conforms to the ROS C++ Style Guide.

## Scan-N-Plan Application: Guidance

### Configure Package 

1. Add a build dependency on `roslint` to your `myworkcell_core` package's `package.xml`:
 
    ``` xml
    <build_depend>roslint</build_depend>
    ```

1. Add `roslint` to the `find_package(...)` command in the `CMakeLists.txt` file:
 
    ``` cmake
    find_package(catkin REQUIRED COMPONENTS
     ...
     roslint
    )
    ```

1. Invoke `roslint` from the `CMakeLists.txt` file

    ``` cmake
    roslint_cpp()
    ```

### Run roslint
1. To run `roslint`:

    ``` bash
    catkin build myworkcell_core --make-args roslint
    ```
1. The output of `roslint` for the ScanNPlan class from Exercise 4.0 indicates that there are several issues regarding formatting

    ``` bash
    /home/ros-industrial/ros/catkin_ws/src/myworkcell_core/src/myworkcell_node.cpp:0:  No copyright message found.  You should have a line: "Copyright [year] <Copyright Owner>"  [legal/copyright] [5]
    /home/ros-industrial/ros/catkin_ws/src/myworkcell_core/src/myworkcell_node.cpp:9:  Single-parameter constructors should be marked explicit.  [runtime/explicit] [5]
    /home/ros-industrial/ros/catkin_ws/src/myworkcell_core/src/myworkcell_node.cpp:54:  Lines should be <= 120 characters long  [whitespace/line_length] [2]
    /home/ros-industrial/ros/catkin_ws/src/myworkcell_core/src/myworkcell_node.cpp:54:  At least two spaces is best between code and comments  [whitespace/comments] [2]
    /home/ros-industrial/ros/catkin_ws/src/myworkcell_core/src/myworkcell_node.cpp:54:  Add #include <string> for string  [build/include_what_you_use] [4]
    Done processing /home/ros-industrial/ros/catkin_ws/src/myworkcell_core/src/myworkcell_node.cpp
    /home/ros-industrial/ros/catkin_ws/src/myworkcell_core/src/vision_node.cpp:0:  No copyright message found.  You should have a line: "Copyright [year] <Copyright Owner>"  [legal/copyright] [5]
    /home/ros-industrial/ros/catkin_ws/src/myworkcell_core/src/vision_node.cpp:12:  Single-parameter constructors should be marked explicit.  [runtime/explicit] [5]
    /home/ros-industrial/ros/catkin_ws/src/myworkcell_core/src/vision_node.cpp:23:  Should have a space between // and comment  [whitespace/comments] [4]
    Done processing /home/ros-industrial/ros/catkin_ws/src/myworkcell_core/src/vision_node.cpp
    Total errors found: 8
    ```

1. Once these issues are corrected, the output of `roslint` should look like this:

    ``` bash
    Done processing /home/ros-industrial/ros/catkin_ws/src/myworkcell_core/src/myworkcell_node.cpp
    Done processing /home/ros-industrial/ros/catkin_ws/src/myworkcell_core/src/vision_node.cpp
    Total errors found: 0
    ```

