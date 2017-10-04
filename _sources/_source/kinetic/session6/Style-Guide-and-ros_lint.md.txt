# ROS Style Guide and ros_lint

## Motivation 
The ROS Scan-N-Plan application is complete, tested and documented.  Now we want to clean up the code according to the style guide so other developers can easily understand our work. 

## Information and Resources 

[The Official ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)

[Automated Style Guide Enforcement](http://wiki.ros.org/roslint)

## Scan-N-Plan Application: Problem Statement
We have completed and tested our Scan-N-Plan program and we need to release the code to the public. Your goal is to ensure the code we have created conforms to the ROS C++ Style Guide. 

## Scan-N-Plan Application: Guidance

### Configure Package 

 1. Add a build dependency on roslint to your package's package.xml:
 
 ``` xml
 <build_depend>roslint</build_depend>
 ```

 2. Add roslint to catkin REQUIRED COMPONENTS in  CMakeLists.txt:
 
 ``` cmake
 find_package(catkin REQUIRED COMPONENTS
   ...
   roslint
 )
 ```

 3. Invoke roslint function from CMakeLists.txt

 ``` cmake
 roslint_cpp()
 ```

### Run roslint
 1. To run roslint:

 ``` bash
 roscd myworkcell_core
 catkin_make --make-args roslint
 ```
