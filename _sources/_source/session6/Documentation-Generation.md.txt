# Documentation Generation

## Motivation
The ROS Scan-N-Plan application is complete and tested.  It is important to thoroughly document the code so that other developers may easily understand this program. 

## Information and Resources

[doxygen generates documentation from annotated source code](http://www.doxygen.org/)

[rosdoc_lite is a ROS wrapper for doxygen](http://wiki.ros.org/rosdoc_lite)

## Scan-N-Plan Application: Problem Statement
We have completed and tested our Scan-N-Plan program and we need to release the code to the public.  Your goal is to make documentation viewable in a browser.  You may accomplish this by annotated the myworkcell_core package with doxygen syntax and generating documentation with rosdoc_lite. 

## Scan-N-Plan Application: Guidance
### Annotate the Source Code

 1. Open the myworkcell_node.cpp file from the previous example. 

 2. Annotate above the ScanNPlan Class:

 ``` c++
/**
* @brief The ScanNPlan class is a client of the vision and path plan servers.  The ScanNPLan class takes
* these services, computes transforms and published commands to the robot.
*/
class ScanNPlan
 ```

 3.  Annotate above the start method

 ``` c++
  /**
   * @brief start performs the robot alorithms functions of the ScanNPlan of
   * the node. The start method makes a service request for a transform that
   * localizes the part.  The start method moves the "manipulator"
   * move group to the localization target.  The start method requests
   * a cartesian path based on the localization target.  The start method
   * sends the cartesian path to the actionlib client for execution, bypassig
   * MoveIt!
   * @param base_frame is a string that specifies the reference frame
   * coordinate system.
   */
void start()
 ```
 4. Annotate above the flipPose

 ``` c++
  /**
   * @brief flipPose rotates the input transform by 180 degrees about the
   * x-axis
   * @param in geometry_msgs::Pose reference to the input transform
   * @return geometry_msgs::Pose of the flipped output transform
   */
geometry_msgs::Pose transformPose(const geometry_msgs::Pose& in) const
 ```
 5. Annotate above the main function

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

 6. Additional annotations may be  placed above private variables or other important code elements.

### Generate documentation

 1. Install rosdoc_lite:
```
sudo apt install ros-kinetic-rosdoc-lite
```

 2. Build the package so we can source it later:

 ```
catkin build
 ```

 3. Source the package

 ```
source ./devel/setup.bash
 ```

 4. run rosdoc_lite to generate the documentation

 ```
roscd myworkcell_core
rosdoc_lite .
 ```

### View the Documentation

 1.  Open the documentation in a browser:

 ```
firefox doc/html/index.html
 ```
 2. Navigate to Classes -> ScanNPlan and view the documentation.
