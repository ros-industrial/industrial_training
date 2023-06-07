# IDE Setup
> An integrated development environment (IDE) is a text editor with a set of tools for helping you program, like syntax highlighting, autocomplete, and debugging. This section details the installation and setup of two IDEs commonly used for ROS development.

> **_These tutorials assume you are using QT Creator. If you are not sure which IDE to use, we recommend QT Creator._**

## QT Creator
[QT Creator](https://github.com/qt-creator/qt-creator) is a popular cross-platform IDE. ROS-Industrial provides a [ROS QT Creator plug-in](https://github.com/ros-industrial/ros_qtc_plugin) with features like ROS build tool integration, automatic sourcing of workspaces, and templates for commonly created files.

You can easily install the editor bundled with the plug-in from the Snap store [here](https://snapcraft.io/qtcreator-ros). Click the green `Install` button in the top right and click `View in Desktop store`. In the window which opens, install the application. Then you can open the IDE by searching for `QT Creator ROS` on your system.

To make a new ROS project open QT Creator and go to `File -> New Project` in the toolbar. In the resulting pop-up, navigate to `Other Project` in the left section. Select `ROS Workspace` in the middle section. Next you will be prompted to enter the project details. For the project workspace you must select an existing directory, so you may have to use your system's file explorer to create a new directory for your project.

Once the project is created, you may right click on the project name under the `Projects` window on the left side of the GUI to bring up the project context window. Select `Add New`. You can choose `ROS` in the left section and `Package` in the middle section to create a new ROS package.

Once you have a package, you can build the workspace by selecting `Build` in the toolbar then `Build all projects`.

## VS Code
[Visual Studio Code](https://code.visualstudio.com/download) is a Microsoft editor which can be used for ROS projects. Download the editor and install the C/C++ extension pack and the ROS extension pack. (Make sure you get the right ones - both extensions are authored by Microsoft.)

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
