# IDE Setup
> An integrated development environment (IDE) is a text editor with a set of tools for helping you program, like syntax highlighting, autocomplete, and debugging. This section details the installation and setup of two IDEs commonly used for ROS development.

## QT Creator
[QT Creator](https://github.com/qt-creator/qt-creator) is a popular cross-platform IDE. ROS-Industrial provides a [ROS QT Creator plug-in](https://github.com/ros-industrial/ros_qtc_plugin) with features like ROS build tool integration, automatic sourcing of workspaces, and templates for commonly created files.

You can easily install the editor bundled with the plug-in from the Snap store [here](https://snapcraft.io/qtcreator-ros). Click the green `Install` button in the top right and click `View in Desktop store`. In the window which opens, install the application. Then you can open the IDE by searching for `QT Creator ROS` on your system.

## VS Code
[Visual Studio Code](https://code.visualstudio.com/download) is another useful editor for ROS-related files. Download the editor and install the C/C++ extension pack and the ROS extension pack. Both are authored by Microsoft. 

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
