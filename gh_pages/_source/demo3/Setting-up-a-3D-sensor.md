# Setting up a 3D sensor

In this exercise, we will setup an Intel RealSense camera to work with our robotic workcell. In this demo we will use the [Intel RealSense D435](https://software.intel.com/en-us/realsense/d400) though a wide range of 3D sensors could be used to publish a ROS point cloud and allow for seamless integration with the rest of the robotic system. The RealSense ROS package can be found [on the wiki](http://wiki.ros.org/RealSense).


## Installing the camera driver

**TODO: try skipping this  and using next section instead***
Before the RealSense camera can be used on the ROS system, the camera driver must first be installed. If this has already been done on your computer, proceed to the next section.

1. Enable Kernel Sources

```
        wget -O enable_kernel_sources.sh http://bit.ly/en_krnl_src
        bash ./enable_kernel_sources.sh
        sudo apt update
```

2. Upgrade to the Xenial kernel. This will restart your computer
```
        sudo apt-get install linux-image-generic-lts-xenial
        sudo reboot
```

## Installing the RealSense SDK

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


## Installing ROS package

Install librealsense2 - http://wiki.ros.org/librealsense2
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

Install realsense2_camera - http://wiki.ros.org/realsense2_camera

## Calibration



## Running with  real camera and simulated robot