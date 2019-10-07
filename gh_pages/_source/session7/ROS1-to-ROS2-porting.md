# ROS1 to ROS2 porting

## Motivation

Our goal for this exercise is to have you fully port a small ROS1 application into ROS2. We'll be
using the basic training material from sessions 1 and 2 as the initial ROS1 application. This will
provide some exposure to the differences in the basic tools of ROS development including publishers,
subscribers, services, and parameters. We will be using exercise 2.3 as the starting point.

## Sequence of actions

1. Create a new workspace

1. Get source dependencies

1. Create new ROS2 packages

1. Test build

1. Copy ROS1 code into ROS2 packages

1. Port the vision node

  1. Class as node and main function

  1. Subscriber

  1. Service (call manually)

1. Port the application node

  1. Node structure and main function

  1. Service client (harcoded parameter)

  1. Runtime parameter

1. Port the launch file


## Setup

1. Create a new ROS2 workspace
 
1. Copy the exercise 2.3 folder and rename it 7.1

1. Inside the src folder, open the myworkcell_core package. 

## CMakeLists.txt

1. Open and start editing the `CMakeLists.txt` file.

1. Start by changing the `cmake_minimuim_required` from `VERSION 2.8.3` to `VERSION 3.5`

1. Next, the required C++ standard for compilation is now C++ 14. 
    ``` 
    if(NOT CMAKE_CXX_STANDARD)
      set(CMAKE_CXX_STANDARD 14)
    endif()
    ```
1. Next, each find_package(X) will need to be separate in ROS2. Also, a few packages need to be changed, `catkin` is replaced with `ament_cmake`, `roscpp` is replaced with `rclcpp`, and, `rosidl_default_generators` needs to be added to the list of packages.
    ``` 
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(fake_ar_publisher_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    ```
1. Next, replace `add_message_files` and `add_service_files` with `rosidl_generate_interfaces`
    ```
    rosidl_generate_interfaces(myworkcell_core
        "srv/LocalizePart.srv"
        DEPENDENCIES geometry_msgs
    )
    ```
1. Declare the C++ executables and the ament dependencies. We will also declare the target_interfaces for the nodes here.
    ```
    add_executable(myworkcell_node src/myworkcell_node.cpp)

    ament_target_dependencies(myworkcell_node rclcpp fake_ar_publisher_msgs)

    add_executable(vision_node src/vision_node.cpp)

    ament_target_dependencies(vision_node rclcpp fake_ar_publisher_msgs)

    rosidl_target_interfaces(myworkcell_node
        ${PROJECT_NAME} "rosidl_typesupport_cpp")

    rosidl_target_interfaces(vision_node
        ${PROJECT_NAME} "rosidl_typesupport_cpp")
    ```
1. Next, we will need to add the include_directories, libraries, and mark install targets
    ``` 
    include_directories(myworkcell_node PUBLIC
                            ${rclcpp_INCLUDE_DIRS}
                            ${fake_ar_publisher_msgs_INCLUDE_DIRS}
                            ${std_msgs_INCLUDE_DIRS})

    ## Specify libraries to link a library or executable target against
    target_link_libraries(myworkcell_node
                        ${rclcpp_LIBRARIES}
                        ${std_msgs_LIBRARIES})

    ## Mark executables and/or libraries for installation
    install(TARGETS 
        myworkcell_node
        vision_node
        DESTINATION lib/${PROJECT_NAME}
    )
    ```
1. Lastly,at the end of the file ament_package() must be called instead of catkin_package()
    ```
    # ament_package is called after the targets
    ament_export_dependencies(rosidl_default_runtime rclcpp)
    ament_package()
    ```

## package.xml

1. Open and start editing the `package.xml` file. Only a few changes need to be made to this file.

1. The package format should be changed to "3"

1. The `<buildtool_depend>` needs to be changed `ament_cmake`

1. Add the package dependencies
    ```
    <depend>rclcpp</depend>
    <depend>fake_ar_publisher_msgs</depend>
    <depend>geometry_msgs</depend>
    ```

1. Inside the export tag, `<build_type>` tag needs to be added with ament_cmake
    ``` xml
    <export>
        <build_type>ament_cmake</build_type>
    </export>
    ```

 ## myworkcell_node.cpp

1. Inside the src folder, open myworkcell_node.cpp to start porting it over. The main changes will consist of changing headers and     changing from using the `roscpp` library API to the `rclcpp` library API.

1. Start by changing the headers. ROS2 requires headers to end in .hpp instead of .h and includes for msgs and srv require the file 
    name to file/srv/ and file/msg/ respectively in their names.
    ```diff
    - #include <ros/ros.h>
    + #include <rclcpp/rclcpp.hpp>
    - #include <myworkcell_core/LocalizePart.h>
    + #include <myworkcell_core/srv/localize_part.hpp>
    ```
1. We will create a ScanNPlan class which will inherit from Node.
    
    ```c++
    class ScanNPlan : public rclcpp::Node
    {
    public:
        explicit ScanNPlan()
        : Node("scan_n_plan")
        {
        vision_client_ = this->create_client<myworkcell_core::srv::LocalizePart>("localize_part");
        }

        ...

    private:
        rclcpp::Client<myworkcell_core::srv::LocalizePart>::SharedPtr vision_client_;
    }
    ```

1. The start function will require a few changes as well. 

    ```c++
    void start(const std::string& base_frame)
    {
        using namespace std::chrono_literals;
        
        // Need to wait until vision node has data
        rclcpp::sleep_for(1s);

        RCLCPP_INFO(this->get_logger(), "Attempting to localize part in frame: %s", base_frame.c_str());

        // The vision client needs to wait until the service appears
        while (!vision_client_->wait_for_service(500ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        }

        // Create a request for the LocalizePart service call
        auto request = std::make_shared<myworkcell_core::srv::LocalizePart::Request>();
        // The base_frame that is passed in is used to fill the request
        request->base_frame = base_frame;

        auto future = vision_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::executor::FutureReturnCode::SUCCESS)
        {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive LocalizePart service response");
        return;
        }

        auto result = future.get();
        if (! result->success)
        {
        RCLCPP_ERROR(this->get_logger(), "LocalizePart service failed");
        return;
        }

        RCLCPP_INFO(this->get_logger(), "Part Localized:  w: %f, x: %f, y: %f, z: %f",
            result->pose.orientation.w,
            result->pose.position.x,
            result->pose.position.y,
            result->pose.position.z);
    }
    ```

1. Lastly, we need to modify the contents of main

    ``` c++
    // This must be called before anything else ROS-related
    rclcpp::init(argc, argv);

    // Create the ScanNPlan object and node
    auto app = std::make_shared<ScanNPlan>();

    // String to store the base_frame parameter after getting it from the Node's parameter client
    std::string base_frame;
    app->declare_parameter("base_frame", "");
    app->get_parameter("base_frame", base_frame);

    if (base_frame.empty())
    {
        RCLCPP_ERROR(app->get_logger(), "No 'base_frame' parameter provided");
        return -1;
    }

    RCLCPP_INFO(app->get_logger(), "ScanNPlan node has been initialized");

    // Call the vision client's LocalizePart service using base_frame as a parameter
    app->start(base_frame);

    rclcpp::shutdown();
    return 0;

    ```
   
## vision_node.cpp

1. Open and start porting over the vision_node.cpp file.

1. Again, start by changing the headers

    ``` c++
    #include <rclcpp/rclcpp.hpp>
    #include <fake_ar_publisher_msgs/msg/ar_marker.hpp>
    #include "myworkcell_core/srv/localize_part.hpp"
    #include <iostream>
    ```

1. We will also again have the object inherit from node

## myworkcell_support

1. The myworkcell_support package will mainly consist of the launch file which is now is in python as well as the accompanying CMakeLists.txt and package.xml

1. Start by renaming the `workcell.launch` file to `workcell.launch.py`

1. Modify the contents to add python headers and the necessary python functions

    ``` py
    from launch import LaunchDescription
    import launch_ros.actions

    def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
        node_name='fake_ar_publisher_node',
        package='fake_ar_publisher',
        node_executable='fake_ar_publisher_node',
        output='screen',
        parameters=[{'x': -0.6},
                    {'y': 0.4},
                    {'z': 0.1},
                    {'camera_frame': 'camera_frame'}],
        ),
        launch_ros.actions.Node(
        node_name='vision_node',
        package='myworkcell_core',
        node_executable='vision_node',
        output='screen',
        ),
        launch_ros.actions.Node(
        node_name='myworkcell_node',
        package='myworkcell_core',
        node_executable='myworkcell_node',
        output='screen',
        parameters=[{'base_frame': 'world'}],
        )
    ])
    ```
1. Make the necessary changes to the CMakesList.txt

    ``` 
    cmake_minimum_required(VERSION 3.10)
    project(myworkcell_support)
    
    # Default to C++14
    if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 14)
    endif()
    
    find_package(ament_cmake REQUIRED)
    find_package(myworkcell_core REQUIRED)
    
    # Install launch files.
    install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
    )

    ament_package()
    ```
1. Make the necessary changes to the package.xml

    ``` xml
    <?xml version="1.0"?>
    <package format="3">
      <buildtool_depend>ament_cmake</buildtool_depend>
      <build_depend>myworkcell_core</build_depend>
      <build_export_depend>myworkcell_core</build_export_depend>
      <exec_depend>myworkcell_core</exec_depend>
    <export>
        <build_type>ament_cmake</build_type>
    </export>
    </package>
    ```

