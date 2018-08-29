# Package Setup
In this exercise, we'll build our package dependencies and configure the package for the Qt Creator IDE. 

## Build Package Dependencies
In a terminal type:
```
cd ~/optimized_planning_ws
catkin build --cmake-args -G 'CodeBlocks - Unix Makefiles'
source devel/setup.bash
```
## Import Workspace into QT
1) In QT do the following:
```
File -> New -> Other Project -> ROS Workspace
```
2) Fill the Project Name and Location information

* Name:  ```pick_and_place```
* Distribution: ``kinetic```
* Build System: ```CatkinTools```
* Workspace Path ```~/optimized_planning_ws```

3) Click next to go to the summary and leave version control to defaults


