# Package Setup
>In this exercise, we'll build our package dependencies and configure the package for the Qt Creator IDE. 

## Build Package Dependencies
In a terminal type:
```
cd ~/optimized_planning_ws
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles'
source devel/setup.bash
```
## Import Package into QT
In QT do the following:
```
File -> New -> Import ROS Project -> 
```

## Open the Main Thread Source File
  In the project tab, navigate into the `[Source directory]/demo3/src/` directory and open the `test_bed_core_node.cpp` file
