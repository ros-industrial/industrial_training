# Setting up a 3D sensor

In this exercise, we will setup an Intel RealSense camera to work with our robotic workcell. In this demo we will use the [Intel RealSense D435](https://software.intel.com/en-us/realsense/d400) though a wide range of 3D sensors could be used to publish a ROS point cloud and allow for seamless integration with the rest of the robotic system. The RealSense ROS package can be found [on the wiki](http://wiki.ros.org/RealSense).


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
```

Test the install by connecting the RealSense camera and running the viewer utility in the terminal.
```
realsense-viewer
```

If problems arise during this process, see the [librealsense documentation](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md).

## Installing ROS package

The ROS package to interface with the RealSense camera can be found in the realsense directory in your workspace. This was cloned [from github](https://github.com/intel-ros/realsense) when you ran ```wstool update```. At this point, the package should already have been built. If not, run ```catkin build```

Run the demo launch file to verify that the ROS interface is working. You should see an RGBD image displayed in RVIZ.
```
roslaunch realsense2_camera demo_pointcloud.launch
```

## Calibration



## Running with  real camera and simulated robot
