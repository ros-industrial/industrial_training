# Setting up a 3D sensor

In this exercise, we will setup an Intel RealSense camera to work with our robotic workcell. In this demo we will use the [Intel RealSense D415](https://software.intel.com/en-us/realsense/d400) though a wide range of 3D sensors could be used to publish a ROS point cloud and allow for seamless integration with the rest of the robotic system. The RealSense ROS package can be found [on the wiki](http://wiki.ros.org/RealSense).


## Installing the RealSense SDK

Check your Ubutnu kernel and ensure that it is 4.4, 4.10, 4.13, or 4.15
```
uname -r
```

 Register the server's public key
```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
```

 Add the server to the list of repositories

```
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
```

Install RealSense demos and utilities
```
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

Test the install by connecting the RealSense camera and running the viewer utility in the terminal. 
```
realsense-viewer
```

Insure the the camera is being recognized as a USB 3.0+ device. If that is not the case, verify that the USB cable used is a 3.0 cable. It has also been found that for long usb cables (>3 m), using an external USB card helps reliability. Regardless, occasional power cycling (unplugging and replugging the USB) may be required.
If problems arise during this process, see the [librealsense documentation](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).

## Installing ROS package

The ROS package to interface with the RealSense camera now needs to be cloned into your workspace. To do this, navigate into your ***/src*** directory and run the following.
```
git clone https://github.com/intel-ros/realsense.git -b development
```

Next, build your workspace.
```
catkin build 
```
If errors occur, make sure that the RealSense SDK was correctly installed.

Run the demo launch file to verify that the ROS interface is working. You should see an RGBD image displayed in RVIZ.
```
roslaunch realsense2_camera demo_pointcloud.launch
```

## Calibration

### The General Idea
Calibrating a camera in your workspace typically happens in two steps:

 1. Calibrate the "intrinsics", the camera sensor & lens, using something like ROS' [camera calibration](http://wiki.ros.org/camera_calibration) package.
 2. Armed with the intrinsics, calibrate the "extrinsics", or the pose of the camera in your workcell.

In our case, the RealSense intrinsics have been calibrated at the factory. This leaves only the second step.

### Terminology
 - **Extrinsic Parameters**: "the extrinsic parameters define the position of the camera center and the camera's heading in world coordinates" [\[ref\]](https://en.wikipedia.org/wiki/Camera_resectioning#Extrinsic_parameters). An extrinsic calibration thus tries to find WHERE your camera is relative to some frame of reference, usually the base of a robot or the wrist of a robot.
 - **Intrinsic Parameters**: When talking about cameras, these parameters define *how* points in 3D space are projected into a camera image. They encompass internal properties of the camera sensor and lens such as focal length, image sensor format, and principal point. An intrinsic calibration tries to solve these
 - **Rectified Image**: Real world cameras and their lenses are NOT perfectly described by the commonly used pinhole model of projection. The deviations from that model, called *distortions*, are estimated as part of *intrinsic calibration* and are used in software to produce an "undistorted" image called the *rectified image*. In our case, the RealSense driver will do this for us.

### Setting up Calibration

The calibration for this exercise is based off of an [AR Tag](http://wiki.ros.org/ar_track_alvar). The AR Tag can be found in ***pick_and_place_support/config***. Print it off then measure and make note of the actual, printed size of the black square. 

Next, we need to place the AR tag at a known location relative to the robot. Since we are still simulating the robot, we will do this by aligning the AR tag with the hole pattern on the bottom of the workcell. Note that the holes are on a 2" hole spacing, and the measurements are taken from the center of the iiwa base. The default values are for the center of the AR tag to be placed 12" in front of the robot with positive x in the long dimension of the workcell.

### Performing the Extrinsic Calibration


With the hardware set up, it's time to perform the calibration. The code has been written for you in pick_and_place_support/src/extrinsic_calibration.cpp.

Open ```calibration.launch``` in the pick_and_place_support package. Notice the first 5 arguments. ```sim_robot``` is a flag to set whether or not we will use the robot to locate the target or measure manually. ```targ_x``` is location of the target when performing the calibration. ```marker_size``` is the size of the black square measured previously in cm.

* Measure the location of the center of the AR tag in meters. Note that x is in the direction of the long dimension of the workcell and z is up with y defined to follow the right hand rule.

* If more than one camera is connected, the serial number of the camera to be calibrated must be provided. The serial numbers of all connected cameras can be found by using ```rs_enumerate_devices```.

* Launch the calibration script filling the values for the target's location and size

```roslaunch pick_and_place_support calibration.launch sim_robot:=true targ_x:=[fill_value] targ_y:=[fill_value] targ_z:=[fill_value] marker_size:=[fill_value] serial_no_camera:=[fill_value] ```

*  Record the output. The script should show the camera as a TF floating in space. After 60 iterations, the program will pause and display the calibration result in the format ```x y z roll pitch yaw```. When you are satisfied that the camera location appears to match reality, copy this pose.



* Update the camera location in pick_and_place/launch/pick_and_place.launch. Lines 17 and 18 publish the location of the camera for the system. Update these numbers with the values from calibration (they are in the same ```x y z yaw pitch roll``` format.



## Running with  real camera and simulated robot

With the camera calibrated, it is time to run the system with a simulated robot but real depth camera. Since our code has already been tested in simulation, this is quite easy. First, update the camera serial numbers in ***pick_and_place/launch/bringup_realsense.launch***. Then simply launch the same file as before but set the sim_sensor flag to false

```roslaunch pick_and_place pick_and_place.launch sim_sensor:=false```










