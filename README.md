industrial_training
===================

Training material for ROS-Industrial.


## IO dependencies installation
===================
The following instructions are base on the guidelines provided in :
https://github.com/orocos/rtt_ros_integration

### Build orocos from source

```shell
export OROCOS_TARGET=gnulinux
mkdir -p ~/ros/hydro/catkin_underlay_isolated/src/orocos
cd ~/ros/hydro/catkin_underlay_isolated
git clone --recursive git://gitorious.org/orocos-toolchain/orocos_toolchain.git -b toolchain-2.7 src/orocos/orocos_toolchain
catkin_make_isolated --install
```

Then create the 'devel' directory by running the following:
```shell
catkin_make
```

This will fail to build since there are non-catkin packages in this repo.  This is
ok since the 'devel' directory and the setup scripts will be created regardless.

### Build rtt-ros-integration from source

```shell
mkdir -p ~/ros/hydro/catkin_underlay/src
cd ~/ros/hydro/catkin_underlay
git clone https://github.com/orocos/rtt_ros_integration.git src/rtt_ros_integration
source ../catkin_underlay_isolated/devel/setup.sh
catkin_make --jobs=2
source devel/setup.sh
```

### Add workspace to your environment setup
In your .bashrc script add the following line:

```shell
source "~/ros/hydro/catkin_underlay/devel/setup.sh"
```
