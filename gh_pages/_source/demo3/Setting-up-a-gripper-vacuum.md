# Setting up a Gripper - Suction Cup

For this tutorial, we will use a suction cup gripper. In order to control it, we need to toggle a solenoid with a GPIO module. We will use the Acromag 951ELN-4012, but the approach would be similar for a wide variety of Modbus controlled GPIO modules.

Note that this tutorial is more advanced than some of the previous tutorials. While examples and guidelines will be given, the "solution" is not provided. Be sure to refer to the prior tutorials as well as the ROS wiki for more information. Additionally, searching any error messages is always a good way to utilize the vast ROS community knowledgebase.

## Setup Network
* Change the IP address
* Connect to the module using a static IP address such as 128.1.1.2
* Open a browser and enter 128.1.1.100
    * Username: User
	* Password: password00
* Go to network configuration and change the following
    * Static IP: 160.69.69.40
	* Subnet mask: 255.255.0.0
	* DNS Server: blank


## Hardware setup
***Important Note:*** These instructions are for a specific setup. Use them only as guidelines. Consult the documentation for your hardware before attempting 
### Wire the solenoid valve
The solenoid input needs to be wired to one of the digital outputs of the gpio module.

### Wire the GPIO module.
This gpio module needs needs external power. Wire in a DC source between 15-36V to the DC+ and DC- ports of the gpio module.

Additionally, the digital outputs need a dc source wired in as well. Wire in a DC source between 5.5-35V to the EXC+ and RTN ports of gpio module

## Make Modbus interface
The gpio module being used for this tutorial communicates using Modbus. While this tutorial will not cover the details of Modbus, we can use ROS to communicate with it without an indepth knowledge of Modbus.

### Clone the Modbus package into your workspace

```
git clone https://github.com/HumaRobotics/modbus.git
```

### Create Modbus Interface node
Next we need to create the node that will send the commands to the gpio module. This is simply modified from the example given in the package cloned above. To do this, create a python node in the pick_and_place/src directory (`touch my_node.py`). Make sure that it is executable (either using `chmod +x filename` or by right clicking on the file)

Next copy in the code below. Note that the registers are from the documentation for this module.
```
#!/usr/bin/env python
########################################################################### 
# This software is graciously provided by HumaRobotics 
# under the Simplified BSD License on
# github: git@www.humarobotics.com:baxter_tasker
# HumaRobotics is a trademark of Generation Robots.
# www.humarobotics.com 

# Copyright (c) 2013, Generation Robots.
# All rights reserved.
# www.generationrobots.com
#   
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation 
#  and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS 
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
# THE POSSIBILITY OF SUCH DAMAGE. 
# 
# The views and conclusions contained in the software and documentation are 
# those of the authors and should not be interpreted as representing official 
# policies, either expressed or implied, of the FreeBSD Project.

import rospy
from modbus.modbus_wrapper_client import ModbusWrapperClient 
from std_msgs.msg import Int32MultiArray as HoldingRegister

# This is the register to control the 951EN-4012 Digital Out
NUM_REGISTERS = 1
ADDRESS_WRITE_START = 102

if __name__=="__main__":
    rospy.init_node("modbus_client")
    rospy.loginfo("""
    This file shows the usage of the Modbus Wrapper Client to write to a register.
    To see the read registers of the modbus server use: rostopic echo /modbus_wrapper/input
    To see sent something to the modbus use a publisher on the topic /modbus_wrapper/output
    """)
    host = "160.69.69.40"
    port = 502

    # setup modbus client    
    modclient = ModbusWrapperClient(host,port=port,rate=50,reset_registers=False,sub_topic="modbus_wrapper/output",pub_topic="modbus_wrapper/input")
    modclient.setWritingRegisters(ADDRESS_WRITE_START,NUM_REGISTERS)
    rospy.loginfo("Setup complete")
    
    # start listening to modbus and publish changes to the rostopic
    modclient.startListening()
    rospy.loginfo("Modbus listener started. Publish binary mask on modbus_wrapper/output to set I/O")
        
    # We are now listening. If anything is sent to the topic, it will be sent via Modbus to the register above.
    while not rospy.is_shutdown():
        rospy.sleep(0.1)

    # Stops the listener on the modbus when ROS shuts down
    modclient.stopListening()
    
```

### Create C++ Example
Next we will create a C++ example. First we need to add an executable to the CMakeLists.txt

```
add_executable(gpio_interface_node src/gpio_interface_node.cpp)
target_link_libraries(gpio_interface_node ${catkin_LIBRARIES})
```

Now create the gpio_interface_node.cpp file in the location specified in the CMakeLists.txt


```
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_gpio");
  ros::NodeHandle nh;

  ros::Publisher modbus_pub = nh.advertise<std_msgs::Int32MultiArray>("modbus_wrapper/output", 1000);

  std_msgs::Int32MultiArray msg;
  while( ros::ok())
  {
    // Set all IO to active (high) 0b111111
    std::vector<int> all_active = {63};
    msg.data = all_active;
    modbus_pub.publish(msg);
    ros::Duration(1.0).sleep();

    // Set all IO to inactive (low) 0b000000
    std::vector<int> all_inactive = {0};
    msg.data = all_inactive;
    modbus_pub.publish(msg);
    ros::Duration(1.0).sleep();

    // Set some IO active - some inactive 0b000111
    std::vector<int> half_active = {7};
    msg.data = half_active;
    modbus_pub.publish(msg);
    ros::Duration(1.0).sleep();
  }

  //exit
  return 0;
}
```

### Test Example
To test this example, first start a roscore and the Python modbus interface node. Then start the C++ example. Verifiy that it is working - switching the IO on and off every second. If the solenoid is wired correctly, you will likely be able to hear it triggering. 

You can now take this code and add it to `test_bed_core_node.cpp` at the appropriate times in order to pick and place the box. While some headers should go at the top, the majority of the code should go in the execution section at the bottom. Look for 
```
// Put gripper code here
// End gripper code here
```







