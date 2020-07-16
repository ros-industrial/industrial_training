# PC Setup

>There are two options for utilizing the ROS-Industrial training materials.  The first **recommended** option is to utilize a pre-configured virtual machine.  The second option is to install a native Ubuntu machine with the required software.  The virtual machine approach is by far the easiest option and ensures the fewest build errors during training but is limited in its ability to connect to certain hardware, particularly over USB (i.e. kinect-like devices).  For the perception training a .bag file is provided so that USB connection is not required for this training course.

## Virtual Machine Configuration (**Recommended**)

The VM method is the most convenient method of utilizing the training materials:

 1. [Download virtual box](https://www.virtualbox.org/wiki/Downloads)
 1. [Download ROS Melodic training VM](https://rosi-images.datasys.swri.edu)
 1. [Import image into virtual box](https://www.virtualbox.org/manual/ch01.html#ovf)
 1. Start virtual machine
    1. *Note: If possible, assign two cores in Settings>>System>>Processor to your virtual machine before starting your virtual machine. This setting can be adjusted when the virtual machine is closed and shut down.
 1. Log into virtual machine, user: ```ros-industrial```, pass: ```rosindustrial``` (no spaces or hyphens)
 1. Get the latest changes (Open Terminal).

    ```
    cd ~/industrial_training
    git fetch origin
    git checkout melodic
    git pull
    ./.check_training_config.bash
    ```

### Limitations of Virtual Box
The Virtual Box is limited both in hardware capability(due to VM limitations) and package installs (to save space).  Kinect-based demos aren't possible due to USB limitations.   

### Common VM Issues
On most new systems, Virtual Box and VMs work out of the box.  The following is a list of issues others have encountered and solutions:
  * Virtualization must be enabled - Older systems do not have virtualization enabled (by default).  Virtualization must be enabled in the BIOS.  See <http://www.sysprobs.com/disable-enable-virtualization-technology-bios> for more information.

## Direct Linux PC Configuration
An installation [shell script](https://github.com/ros-industrial/industrial_training/blob/melodic/gh_pages/_downloads/ros-industrial-training-setup.sh)
is provided to run in Ubuntu Linux 18.04 LTS (Bionic). This script installs ROS and any other packages needed for the environment used for this training.

After this step (or if you already have a working ROS environment), clone the training material repository into your home directory:

`git clone -b melodic https://github.com/ros-industrial/industrial_training.git ~/industrial_training`

## Configuration Check
The following is a quick check to ensure that the appropriate packages have been installed and the the `industrial_training` git repository is current.  Enter the following into the terminal:

`~/industrial_training/.check_training_config.bash`
