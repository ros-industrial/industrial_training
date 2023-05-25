# PC Setup

>There are two options for utilizing the ROS-Industrial training materials.  The first **recommended** option is to utilize a pre-configured virtual machine.  The second option is to install a native Ubuntu machine with the required software.  The virtual machine approach is by far the easiest option and ensures the fewest build errors during training but is limited in its ability to connect to certain hardware, particularly over USB (i.e. kinect-like devices).  For the perception training a .bag file is provided so that USB connection is not required for this training course.

## Virtual Machine Configuration (**Recommended**)

The VM method is the most convenient method of utilizing the training materials:

 1. [Download virtual box](https://www.virtualbox.org/wiki/Downloads)
 1. [Download ROS Foxy training VM](https://rosi-images.datasys.swri.edu)
 1. [Import image into virtual box](https://www.virtualbox.org/manual/ch01.html#ovf)
 1. Start virtual machine
    1. *Note: If possible, assign two cores in Settings>>System>>Processor to your virtual machine before starting your virtual machine. This setting can be adjusted when the virtual machine is closed and shut down.
 1. Log into virtual machine, user: ```ros-industrial```, pass: ```rosindustrial``` (no spaces or hyphens)
 1. Get the latest changes (Open Terminal).

    ```
    cd ~/industrial_training
    git fetch origin
    git checkout foxy
    git pull
    ./.check_training_config.bash
    ```

### Limitations of Virtual Box
The Virtual Box is limited both in hardware capability(due to VM limitations) and package installs (to save space).  Kinect-based demos aren't possible due to USB limitations.   

### Common VM Issues
On most new systems, Virtual Box and VMs work out of the box.  The following is a list of issues others have encountered and solutions:
  * Virtualization must be enabled - Older systems do not have virtualization enabled (by default).  Virtualization must be enabled in the BIOS.  See <http://www.sysprobs.com/disable-enable-virtualization-technology-bios> for more information.

## Direct Linux PC Configuration
An installation [shell script](https://github.com/ros-industrial/industrial_training/blob/foxy/gh_pages/_downloads/ros-industrial-training-setup.sh)
is provided to run in Ubuntu Linux 20.04 LTS (Focal). This script installs ROS and any other packages needed for the environment used for this training.

After this step (or if you already have a working ROS environment), clone the training material repository into your home directory:

`git clone -b foxy https://github.com/ros-industrial/industrial_training.git ~/industrial_training`

## Configuration Check
The following is a quick check to ensure that the appropriate packages have been installed and the the `industrial_training` git repository is current.  Enter the following into the terminal:

`~/industrial_training/.check_training_config.bash`

## VS Code Integration
[Visual Studio Code](https://code.visualstudio.com/download) is a useful editor for ROS-related files. Download the editor and install the C/C++ extension pack and the ROS extension pack. Both are authored by Microsoft. 

After opening a workspace folder, navigate to the workspace settings (`Ctrl+shift+P -> "Preferences: Open workspace settings (JSON)"`). Add the key `"ros.distro": "<distro>"`. For example, 

```
{
   "ros.distro": "foxy",
}
```

Upon opening a C++ file, you should get a pop-up offering to configure the workspace settings. After accepting, you should have a file `.vscode/c_cpp_properties.json` similar to

```
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "gnu++14",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```

Add the appropriate ROS and user include paths to resolve IDE include errors. For example,

```
"includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/foxy/include/**",
                "/usr/include/**",
            ],
```

where `foxy` can be changed to the appropriate distro.

Now you're prepared to work on ROS projects in VS Code.
