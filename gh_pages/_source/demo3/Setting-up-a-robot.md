# Setting-up-a-robot.md

Next, we will move to using a real robot. For this tutorial, we will be using the KUKA iiwa 7kg collaborative robot. These instructions are primarily taken from the [iiwa_stack wiki](https://github.com/IFL-CAMP/iiwa_stack/wiki), and it is recommended that the user read this wiki before following these instructions. Further, as with any hardware integration task, it is likely that there will be aspects specific to your setup that may not be addressed here. As always, this is where the large ROS community knowledge base is here to help. Commonly, solutions to such problems can be found in [ROS Answers](https://answers.ros.org/questions/) or in the driver repo's [issue tracker](https://github.com/IFL-CAMP/iiwa_stack/issues).


## **Introduction**

The **KUKA LBR IIWA** redundant manipulator is programmed using the KUKA's Sunrise Workbench platform and its Java APIs.   
A **Sunrise project**, containing one or more **Robotic Application** that can be synchronized to the robot cabinet and executed from the **SmartPad**.    

Within this software package, you will find a **Robotic Application** that can be used with the robot.   
It establishes a connection with an additional machine connected via Ethernet to the robot cabinet via ROS.   
The additional machine, having ROS installed, will be able to send and receive ROS messages to and from the aforementioned **Robotic Application**.   
We make use of ROS messages already available in a mint ROS distribution as well as custom ones, made to work with the KUKA arm (*iiwa_msgs*).    
A user is then able to manipulate the messages received from the robot and to send new ones as commands to it, simply within C++ or python code, taking leverage of all the ROS functionalities.

In this wiki, we will refer as **ROSCORE** side for the additional machine equipped with ROS and **SUNRISE** side for anything included in robot cabinet.

## **Setup Guide**

> IMPORTANT : A version of **Sunrise Workbench** with the **Sunrise.Connectivity** module is required to use this stack. If you have something referring to "Smart Servoing", that is the correct module. Connectivity has been tested on Sunrise versions: 1.5, 1.7, 1.9, 1.11, 1.13, 1.14, and 1.15. If you don't have the Connectivity module, you might want to ask KUKA for it.

> Whatever version you use, the Sunrise OS, Sunrise workbench, and Connectivity module must all be compatible.


### Sunrise Cabinet setup
### Cabinet setup

A good part of this page is taken from [Khansari's wiki][1]. In case that one will go down one day, here it is.

Before we start, this is a good time to make a factory image of your cabinet. If you purchased the KUKA recovery stick, this will allow you to restore the cabinet in case something goes wrong. Unfortunately the recovery stick does not come with a factory image, so make one before you get started. To do that, connect a monitor and keyboard to the cabinet then simply turn off the cabinet, insert the usb, and reboot the cabinet. It will boot to the recovery menu. You will likely need to change the language using the Sprachauswahl button. Select create image and give it a name. When you're done shut down the cabinet, remove the stick, and reboot.

The Cabinet of an IIWA robot has multiple ethernet ports, from here after, we are gonna make use of the KONI interface.

> NOTE: The KONI Ethernet port is on the left side of the front panel of the cabinet. There should be a label on top of it.     

The KONI interface by default belongs to the Real Time OS on the Cabinet, and available basically only for FRI.     
To use it, we need to change the configuration of the Cabinet itself.    

> WARNING: Follow these instructions at your own risk. KUKA doesn't provide these instructions, but it's rather a home-brew solution. It may cause problems to your hardware (even if so far none is known).

#### Change the configuration of the KONI interface      

1. **Log in into the Cabinet.**         
There are different methods to have access to the cabinet. The best for this job is to connect the cabinet to external monitor, mouse, and keyboard (USB and DVI ports on the bottom left) and start the cabinet without the SmartPad plugged in.   
The Windows7 Embedded should start up with the _KukaUser_ already logged in, in case not you could search for the credentials of that user, [google is your friend.](http://lmgtfy.com/?q=kuka+admin+password)

2. **Shutdown KRC**     
Right-click on the green icon in the system tray (bottom right, next to the clock) and click Stop KRC.          

3. **Check the Network Interfaces**     
Open the list of the network interfaces: Start -> View network connections.     
At this point you should only have one interface: **Realtime OS Virtual Network Adapter**.     
Just remember it exists and **DO NOT EVER CHANGE ITS SETTINGS**.

3. **Open a terminal and type**        
`C:\KUKA\Hardware\Manager\KUKAHardwareManager.exe -assign OptionNIC -os WIN`    
This will assign the KONI interface to Windows.  

Note: If you are not familiar with the German keyboard, you might want to change it by going to start->Region and Language -> Keyboards and language and selecting your favorite keyboard layout.

A COMMON ISSUE: It is been reported by several users that the above command does not work in some new versions of
Sunrise OS, and gives the following error: "Assignment not possible while KS is running". To handle this issue you
need to remove KS from the windows auto startup. Do this by going to start -> msconfig.exe and disabling BoardPackage and KR C. Restart the PC and now KUKA software should not be loaded. Now apply the command. After that, put back the KS in the Windows auto startup. Restart Windows and you should be good to continue with the remaining steps.

4. **Install the network adapter driver (often not needed)**    
Open the Device Manager, if you don't have any network adapter marked in yellow then just skip this point.     
Else, You can find the driver for it in C:\KUKA\Hardware\Drivers\.        
Sometimes you will find there two folders for different Ethernet controllers, check within the Device Manager which one you need.

5. **Reboot and open Start -> View network connections again**   
Change the settings (IP address) of the **new** Ethernet adapter to the one you want to use - within the Network adapter settings in Windows. Choose a **different** subnet than the one used by Sunrise Workbench, for example, we use IP: *160.69.69.69* and subnet mask: 255.255.0.0.     

**Alternatively**, you could open `C:\Windows\System32\drivers\etc\hosts` as Administrator and add the hostname and IP of the ROS machine you are using. This will allow to get an IP in the subnet via DHCP.

Connect an Ethernet cable between the **KONI** interface and the Ethernet adapter on the **ROSCORE** side.

Further comments from users can be found in some issues: e.g. 
[#65](https://github.com/SalvoVirga/iiwa_stack/issues/65)

**NOTE:**    
Due to this alteration, other applications using the KONI Ethernet Port of the Cabinet won't work anymore.    
For example, any application using the native KUKA's FRI library won't be functioning, as it uses the same Ethernet Port for communication.     
To revert the change execute:   
`C:\KUKA\Hardware\Manager\KUKAHardwareManager.exe -assign OptionNIC -os RTOS`    
this will assign back the KONI interface to the Real Time OS.

[1]: https://bitbucket.org/khansari/iiwa/wiki/Home




### Roscore PC setup

The first 3 steps should already have been completed if you are following this tutorial. However, they are kept here for completeness.

1. **Install ROS KINETIC or INDIGO** (if not already there) as described at http://wiki.ros.org/kinetic/Installation/Ubuntu. (till section 1.7)   
   It's also a good idea to install the python catkin tools     
   `sudo apt-get install python-catkin-tools`

2. **Clone this repository to your workspace** (you can omit the first 3 commands if you already have one) :   
`mkdir ros_ws && cd ros_ws && mkdir src`          
`catkin_init_workspace`   
`git clone https://github.com/SalvoVirga/iiwa_stack.git src/iiwa_stack` 


3. **Download dependences** :      
`rosdep install --from-paths src --ignore-src -r -y`

4. **Build the workspace** :  
`catkin build`

5. **Source workspace** :    
`source devel/setup.bash`

6. **Setup network** :    
Connect the KONI port of the cabinet to the ROS PC. Setup the network such that the ROS PC has a static IP address on the same subnet as the one used by the cabinet. For this example we are using subnet: 255.255.0.0 and IP: 160.69.69.100.

Open   
`gedit ~/.bashrc &`    
and append these two lines at the end     
`export ROS_IP=xxx.xxx.xxx.xxx`      
`export ROS_MASTER_URI=http://$ROS_IP:11311`   
With *xxx.xxx.xxx.xxx* an IP address on the same subnet of the one you used in the [**SUNRISE**](cabinet_setup) setup. Also, configure the network interface - within your Operating System - you are going to use with the same IP address. For example, we use *160.69.69.100*.    

7. **Open a terminal and ping the KONI port of the cabinet**.     
In **our case** : `ping 160.69.69.69`     
if you get an answer, everything is ready.  


## Sunrise Setup

### **Set up your Sunrise Project**

1. Create a Sunrise project, or load an existing one from the controller (default IP: 172.31.1.147).
2. Open the StationSetup.cat file
3. Select the _Software_ tab
4. Enable the Servo Motion packages, this is how that page should look like :
![station][stationsetup]
5. Save and apply changes

Now you will need to add `iiwa_stack` (more precisely the content of `iiwa_ros_java`) to the Sunrise project.

You could either:

- Copy iiwa_ros_java into the Sunrise Project: (changes will not be seen by git)

    1. Copy the content of _iiwa\_stack_/_iiwa\_ros\_java_/_src_ inside the _src_ folder.
    1. Copy the folder _iiwa\_stack_/_iiwa\_ros\_java_/_ROSJavaLib_ into the root of the project.
    1. Inside Sunrise Workbench select all the files inside _ROSJavaLib_, right click and choose _Build Path_ -> Add to Build Path... (you may have to refresh to see them)

or

- Create symlinks to iiwa_ros_java: (changes will be tracked by git!)

    1. Run the command prompt as an administrator (press windows key->type cmd->right click->run as an administrator).
    1. type `cd \path\to\sunrise\project`
    1. type `mklink /D ROSJavaLib ..\relative\path\to\iiwa_stack\iiwa_ros_java\ROSJavaLib`
    1. type `cd src`
    1. type `mklink /D de ..\relative\path\to\iiwa_stack\iiwa_ros_java\src\de`
    1. Inside Sunrise Workbench select all the files inside _ROSJavaLib_, right click and choose _Build Path_ -> Add to Build Path...

❗ __IMPORTANT__ ❗   
6. Have you already set your __Safety Configuration__? If not, you should! [**HERE**](https://github.com/SalvoVirga/iiwa_stack/wiki/safetyconf) we show how we set up ours.         
7. Be sure the right __master_ip__ is set in the _config.txt_ file. See below.

#### **Configuration File**

From release 1.7.x, we introduced a **configuration file** that allows to easily set some parameters.   
In the  __Java__ project you just created, you should have a **config.txt** file.     

There you can set : 
- robot_name: name of your robot, by default *iiwa*. Read more about this [HERE](namingrobot);
- master_ip: IP of the **ROSCORE** machine you just set [HERE](roscore_setup), for example *160.69.69.100*;
- master_port: port for ROS communication, also set [HERE](roscore_setup), by default is *11311*;
- ntp_with_host: turns on/off the ntptimeprovider from ROSJava to sync the two machines. The default value is false, note that you need an NTP provider on the external machine if you want to set it to true.

Once you properly fill this file, **everything is ready**. There should be no red sign or problem found by the IDE.
If this is a new project? Then you first need to _Install_ it (_Station Setup_ -> _Installation_) and then synchronize it.

## Execute trajectory on robot

With setup complete, you are ready to execute on the robot. First, switch the robot into auto mode by turning the key, selecting auto, and turning the key back.

Then select the ROSSMArtServo program on the smart pad and press the run button.

Finally, simply run the same code from before, this time with sim_robot=false.

```roslaunch pick_and_place pick_and_place.launch sim_sensor:=false sim_robot:=false```


The system should use a 3D scan of the environment to find the pick location, path plan the pick and place moves, then ask you if you would like to execute. Simply type y+enter to execute on the robot.

One thing to note is that a ROS node is actually running on the KUKA cabinet PC. This means that pausing the program on the smart pad (e.g. by manually jogging it) or restarting the launch file can cause problems. In order to get the program to reconnect to ROS master, you may need to restart the program by unselecting it and reselecting it in the program menu then rerunning it.

