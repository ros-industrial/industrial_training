# Documentation Generation

## Motivation
The ROS Scan-N-Plan application is complete and tested.  It is important to thoroughly document the code so that other developers may easily understand this program. 

## Information and Resources

[`doxygen` generates documentation from annotated source code](http://www.doxygen.org/)

[`rosdoc_lite` is a ROS wrapper for doxygen](http://wiki.ros.org/rosdoc_lite)

## Scan-N-Plan Application: Problem Statement
We have completed and tested our Scan-N-Plan program from Exercise 4.0 and we need to release the code to the public.  Your goal is to make documentation viewable in a browser.  You may accomplish this by annotated the `myworkcell_core` package with `doxygen` syntax and generating documentation with `rosdoc_lite`.

## Scan-N-Plan Application: Guidance
### Annotate the Source Code

1. Open the myworkcell_node.cpp file from the previous example.

    1. Annotate above the `ScanNPlan` class:

        ``` c++
        /**
        * @brief The ScanNPlan class is a client of the vision and path plan servers.  The ScanNPLan class takes
        * these services, computes transforms and published commands to the robot.
        */
        class ScanNPlan
        ```

    1.  Annotate above the `start` method

        ```c++
        /**
         * @brief start performs the motion planning and execution functions of the ScanNPlan of
         * the node. The start method makes a service request for a transform that
         * localizes the part.  The start method moves the "manipulator"
         * move group to the localization target.  The start method requests
         * a cartesian path based on the localization target.  The start method
         * sends the cartesian path to the actionlib client for execution, bypassig
         * MoveIt!
         * @param base_frame is a string that specifies the reference frame
         * coordinate system.
         */
        void start(const std::string& base_frame)
        ```

    1. Annotate above the `main` function

        ``` c++
        /**
         * @brief main is the ros interface for the ScanNPlan Class
         * @param argc ROS uses this to parse remapping arguments from the command line.
         * @param argv ROS uses this to parse remapping arguments from the command line.
         * @return ROS provides typical return codes, 0 or -1, depending on the
         * execution.
         */
        int main(int argc, char** argv)
        ```

    1. Additional annotations may be  placed above private variables or other important code elements.

### Generate documentation

1. Install `rosdoc_lite`:

    ```
    sudo apt install ros-melodic-rosdoc-lite
    ```

1. Build the workspace so we can produce documentation for its packages later:

     ```
     catkin build
     ```

1. Source the workspace

     ```
     source ./devel/setup.bash
     ```

1. Run `rosdoc_lite` to generate the documentation

    ```
    roscd myworkcell_core
    rosdoc_lite .
    ```

### View the Documentation

1.  Open the documentation in a browser:

    ```
    firefox doc/html/index.html
    ```

1. Navigate to Classes -> ScanNPlan and view the documentation.
