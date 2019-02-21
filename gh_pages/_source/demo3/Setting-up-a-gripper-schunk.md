# Setting up a Schunk Gripper
 
For this tutorial, we will use a Schunk WSG050 gripper. We will communicate with this gripper via Ethernet TCP/IP. However, the ROS driver hides most of the details of this interface, allowing us to control it without indepth knowledge of TCP/IP. Further, it is worth noting that there are at least 2 community supported ROS drivers that provide different interfaces. We will demonstrate only the first one.
 
* https://github.com/ipa320/ipa325_wsg50.git
* https://github.com/nalt/wsg50-ros-pkgS
 
Note that this tutorial is more advanced than some of the previous tutorials. While examples and guidelines will be given, the "solution" is not provided. Be sure to refer to the prior tutorials as well as the ROS wiki for more information. Additionally, searching any error messages is always a good way to utilize the vast ROS community knowledgebase.
 
## Setup Network
* Connect to the gripper using a static IP address such as 192.168.1.2
* Open a browser and enter 192.168.1.20
* Settings → Network → change IP address and subnet mask to
    * IP: 160.69.69.20
    * subnet: 255.255.0.0
 
## Clone the driver package into your workspace
Like many community ROS package, there is not a debian release for this driver. Therefore you will need to clone the source of the package into the src space of your workspace.
```
git clone https://github.com/ipa320/ipa325_wsg50.git
```
 
## Create Example node
### Edit Build Information
Next we will create a C++ example. First we need to add an executable to the CMakeLists.txt
 
```
add_executable(gripper_interface_node src/gripper_interface_node.cpp) 
target_link_libraries(gripper_interface_node ${catkin_LIBRARIES}) 
```
 
Additionally, you will need to add a dependency on the ipa325_wsg50 by adding it to the appropriate places in the CMakeLists.txt
 
* find_package(catkin REQUIRED COMPONENTS [all_of your other packages] ipa325_wsg50)
* catkin_package( CATKIN_DEPENDS [all of your other packages] ipa325_wsg50)
 
Finally, you need to add the following line to your package.xml file
 
```
 <depend>ipa325_wsg50</depend> 
```

 
### Add gripper_interface_node.cpp
 
Now create the gripper_interface_node.cpp file in the location specified in the CMakeLists.txt
 
 
```
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa325_wsg50/WSG50HomingAction.h>
#include <ipa325_wsg50/WSG50GraspPartAction.h>
#include <ipa325_wsg50/WSG50ReleasePartAction.h>
#include <ipa325_wsg50/ackFastStop.h>
 
int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_gripper");
  ros::NodeHandle nh;
 
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<ipa325_wsg50::WSG50HomingAction> home_ac("WSG50Gripper_Homing", true);
  actionlib::SimpleActionClient<ipa325_wsg50::WSG50GraspPartAction> grasp_ac("WSG50Gripper_GraspPartAction", true);
  actionlib::SimpleActionClient<ipa325_wsg50::WSG50ReleasePartAction> release_ac("WSG50Gripper_ReleasePartAction", true);
  ros::ServiceClient srv = nh.serviceClient<ipa325_wsg50::ackFastStop>("/AcknowledgeFastStop");
 
  ROS_INFO("Waiting for action servers to start.");
  home_ac.waitForServer(); //will wait for infinite time
  grasp_ac.waitForServer();
  release_ac.waitForServer();
  ipa325_wsg50::ackFastStop ack;
  srv.call(ack);
 
  ROS_INFO("Homing gripper...");
  ipa325_wsg50::WSG50HomingGoal home_goal;
  home_goal.direction = true;   // True is in the out direction
  home_ac.sendGoal(home_goal);
  bool finished_before_timeout = home_ac.waitForResult(ros::Duration(10.0));
 
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = home_ac.getState();
    ROS_INFO("Homing finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Homing did not finish before the time out.");
 
  ROS_INFO("Grasping Part...");
  ipa325_wsg50::WSG50GraspPartGoal grasp_goal;
  grasp_goal.width = 80;  // Part width in mm
  grasp_goal.speed = 20;  // Speed in mm/s
  grasp_ac.sendGoal(grasp_goal);
  finished_before_timeout = grasp_ac.waitForResult(ros::Duration(15.0));
 
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = grasp_ac.getState();
    ROS_INFO("Grasp finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Grasp did not finish before the time out.");
 
  ROS_INFO("Releasing Part...");
```
 
## Test Example
To test this example, first launch the launch file that was included with the ROS driver (`wsg50.launch`).  Then start the C++ example. Verifiy that it is working - opening and closing the Schunk gripper.
 
You can now take this code and add it to `test_bed_core_node.cpp` at the appropriate times in order to pick and place the box. While some headers should go at the top, the majority of the code should go in the execution section at the bottom. Look for 
```
// Put gripper code here
// End gripper code here
```
 
