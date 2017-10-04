# Package Setup
>In this exercise, we'll build our package dependencies and configure the package for the Qt Creator IDE. 

## Build Package Dependencies
In a terminal type:
```
cd ~/perception_driven_ws
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles'
source devel/setup.bash
```
## Import Package into QT
In QT do the following:
```
File -> New -> Import ROS Project -> 
```

## Open the Main Thread Source File
  In the project tab, navigate into the `[Source directory]/collision_avoidance_pick_and_place/src/nodes` directory and open the `pick_and_place_node.cpp` file
