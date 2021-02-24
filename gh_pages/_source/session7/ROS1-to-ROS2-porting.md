# ROS1 to ROS2 porting

## Introduction

Our goal for this exercise is to have you fully port a small ROS1 application into ROS2. We'll be
using the basic training material from sessions 1 and 2 as the initial ROS1 application. This will
provide some exposure to the differences in the basic tools of ROS development including publishers,
subscribers, services, and parameters. We will be using exercise 2.3 as the starting point.

## Workspace setup

1.  Clone the ROS2 version of `fake_ar_publisher` into your ROS2 workspace

    ```
    cd ~/ros2_ws/src/
    git clone -b ros2 https://github.com/ros-industrial/fake_ar_publisher.git
    ```

1.  Create empty ROS2 packages.

    ```
    cd ~/ros2_ws/src/
    ros2 pkg create myworkcell_core
    ros2 pkg create myworkcell_support
    ```

1.  Verify the workspace builds with `colcon build`

1.  Set up a new project in Qt Creator with 'Build System' set to 'Colcon' and 'Distribution' set to
    '/opt/ros/eloquent'. This is optional but highly recommended since you will get code
    autocompletion when editing C++ in the Qt Creator IDE. Verify that the workspace builds
    successfully from inside the IDE.

1.  Copy the ROS1 code from the packages at `~/industrial_training/exercises/2.3/src/` into the
    empty ROS2 packages.  We'll want everything copied over except `CMakeLists.txt` and
    `package.xml`, where we will be modifying the empty ROS2 versions instead. For more complex
    porting efforts, it will likely be more efficient to directly modify the ROS1 versions of these
    files.

## Porting

We will more or less follow the same sequence that the ROS1 basic training takes in switching to
ROS2.

### Topic subscription

1.  Open and start editing the `CMakeLists.txt` file in `myworkcell_core`

1.  Package dependencies: Ament packages you depend on are now found directly using the CMake
    `find_package` command. Almost always the first invocation will be `find_package(ament_cmake)`
    which is required to make this package an ament package.

    Add additional dependencies for this application below this first `find_package` command:

    ```
    find_package(rclcpp REQUIRED)
    find_package(fake_ar_publisher_msgs REQUIRED)
    ```

1.  Build the vision node: An executable is added in the usual CMake way. Rather than use
    `include_directories` and `target_link_libraries` to depend on other packages, a convenience
    macro is provided to depend on other ament packages.

    ```
    add_executable(vision_node src/vision_node.cpp)
    ament_target_dependencies(vision_node rclcpp fake_ar_publisher)
    ```

1.  Install the vision node: Everything in ROS2 must have an install rule in order to be used at
    runtime.

    ```
    install(TARGETS vision_node
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    ```

1.  Above the final call to `ament_package()` add a line which indicates which ament packages this
    package depends on at runtime:

    ```
    ament_export_dependencies(rclcpp fake_ar_publisher)
    ```

1.  You may try to build the workspace again now and you should get a compile error about
    `ros/ros.h` not being found. We're now ready to port the C++ code. Open `vision_node.cpp` for
    editing.

1.  Change the headers: ROS2 requires headers to end in .hpp instead of .h and includes for msgs and
    srv require the file name to file/msg/ and file/srv/ respectively in their names.

    ```diff
    - #include <ros/ros.h>
    + #include <rclcpp/rclcpp.hpp>
    - #include <fake_ar_publisher/ARMarker.h>
    + #include <fake_ar_publisher/msg/ar_marker.hpp>
    ```

1.  Comment out the callbacks, the member variables, and the contents of the constructor. Change the
    `Localizer` class to inherit from `rclcpp::Node`.

    ```c++
    class Localizer : public rclcpp::Node
    {
    public:
        Localizer()
        : Node("vision_node")
        {
           //...
        }
        //...
    };
    ```

1.  The main function can now be replaced with ROS2 equivalents of creating a node and spinning to
    wait for callbacks to be invoked:

    ```c++
    int main(int argc, char* argv[])
    {
        // This must be called before anything else ROS-related
        rclcpp::init(argc, argv);

        // Create the node
        auto node = std::make_shared<Localizer>();

        // Don't exit the program.
        rclcpp::spin(node);
    }
    ```

1.  Now we'll port the subscriber Replace the `ar_sub_` and `last_msg_` member variables with:

    ```c++
    rclcpp::Subscription<fake_ar_publisher::msg::ARMarker>::SharedPtr ar_sub_;
    fake_ar_publisher::msg::ARMarker::SharedPtr last_msg_;
    ```

1.  Add the subscription creation to the constructor:

    ```c++
    using namespace std::placeholders;

    ar_sub_ = this->create_subscription<fake_ar_publisher::msg::ARMarker>("ar_pose_marker", rclcpp::QoS(1),
      std::bind(&Localizer::visionCallback, this, _1));
    ```

    and the callback that stores the message:

    ```c++
    void visionCallback(fake_ar_publisher::msg::ARMarker::SharedPtr msg)
    {
      last_msg_ = msg;
      RCLCPP_INFO(this->get_logger(), "Received ARMarker message");
    }
    ```

1.  Verify the workspace builds now and test the topic subscription. Run the publisher node and
    vision node in separate terminals:

    ```
    ros2 run fake_ar_publisher fake_ar_publisher
    ros2 run myworkcell_core vision_node
    ```

### Interface generation

We'll now build the `LocalizePart` service type to use in the `myworkcell_core` package.

1.  The `.srv` file syntax is basically unchanged, but we do have to modify the definition. Since
    ROS2 does not allow you to return a boolean from a service callback, we must track success and
    failure ourselves. Add a `success` member to the response in `LocalizePart.srv`:

    ```
    #request
    string base_frame
    ---
    #response
    bool success
    geometry_msgs/Pose pose
    ```

1.  Modify CMake and `package.xml` to build the service. In `CMakeLists.txt` add additional
    dependencies and a call to generate custom interfaces:

    ```
    find_package(rosidl_default_generators REQUIRED)
    find_package(geometry_msgs REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "srv/LocalizePart.srv"
      DEPENDENCIES geometry_msgs
    )
    ```

    To use generated interfaces in the same package, another dependency must be added on built
    targets:

    ```
    rosidl_target_interfaces(vision_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
    ```

    In `package.xml` make sure the `format` version is 3 and add the following tags:

    ```xml
    <member_of_group>rosidl_interface_packages</member_of_group>
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    ```

1.  Make sure the workspace builds and you can now see the `myworkcell_core/srv/LocalizePart`
    service when running `ros2 srv list`.
### Services

1.  At the top of `vision_node.cpp` add/uncomment:

    ```c++
    #include <myworkcell_core/srv/localize_part.hpp>
    ```

    Then add the service server to the vision node, replacing the `ServiceServer` member variable
    with:

    ```c++
    rclcpp::Service<myworkcell_core::srv::LocalizePart>::SharedPtr server_;
    ```

    Add the service creation to the constructor:

    ```c++
    server_ = this->create_service<myworkcell_core::srv::LocalizePart>("localize_part",
      std::bind(&Localizer::localizePart, this, _1, _2));
    ```

    and the service callback:

    ```c++
    void localizePart(myworkcell_core::srv::LocalizePart::Request::SharedPtr req,
                      myworkcell_core::srv::LocalizePart::Response::SharedPtr res)
    {
      // Read last message
      fake_ar_publisher::msg::ARMarker::SharedPtr p = last_msg_;

      if (!p){
        RCLCPP_ERROR(this->get_logger(), "no data");
        res->success = false;
        return;
      }

      res->success = true;
      res->pose = p->pose.pose;
    }
    ```

1.  Run both the `fake_ar_publisher` node and the `vision_node` and try manually calling the
    service.

1.  Now we'll set up the application node to call the service. Add a new target to `CMakeLists.txt`:

    ```
    add_executable(myworkcell_node src/myworkcell_node.cpp)
    ament_target_dependencies(myworkcell_node rclcpp fake_ar_publisher_msgs)
    rosidl_target_interfaces(myworkcell_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
    ```

    and add `myworkcell_node` to the existing `install` function call, after `vision_node`.

1.  Open `myworkcell_node.cpp` to start porting it over. As done with the vision node, convert the
    `ScanNPlan` class to inherit from `rclcpp::Node` and rewrite the `main` function in the same
    way. For now, comment out the code that loads the 'base_frame' parameter and call the `start`
    function with a hardcoded `"world"`.
    
    ```c++
    class ScanNPlan : public rclcpp::Node
    {
    public:
        ScanNPlan() : Node("scan_n_plan")
        {
            //...
        }
        //...
    }

    int main(int argc, char* argv[])
    {
        //...
    }
    ```

1.  Replace the `vision_client` member variable with:

    ```c++
    rclcpp::Client<myworkcell_core::srv::LocalizePart>::SharedPtr vision_client_;
    ```

    and create it in the constuctor with:

    ```c++
    vision_client_ = this->create_client<myworkcell_core::srv::LocalizePart>("localize_part");
    ```

1.  The `start` function will be fully replaced since it is quite a bit more complex. Since ROS2
    services are now asynchronous, the client returns a `future` object when called and the
    developer is responsible for spinning until a response returns from the vision node. Read
    through the new `start` function here before replacing to make sure you understand everything
    that is happening.

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

1.  Verify everything builds and works

### Parameters

1.  Parameters work slightly differently than in ROS1. The most significant change is the parameter
    declaration mechanism, which, when given a default value, ensures that a parameter will exist
    under the given name which matches the type of the default. Modify the `main` function to
    declare and get the `base_frame` parameter:

    ``` c++
    auto node = std::make_shared<ScanNPlan>();

    // String to store the base_frame parameter after getting it from the Node's parameter client
    std::string base_frame;
    node->declare_parameter("base_frame", "");
    node->get_parameter("base_frame", base_frame);

    if (base_frame.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "No 'base_frame' parameter provided");
        return -1;
    }

    // Call the vision client's LocalizePart service using base_frame as a parameter
    node->start(base_frame);
    ```

1.  Unfortunately, parameters can not currently be set on the command line when using `ros2 run`. To
    test that the parameter is set, you must create and save a YAML file:

    ```yaml
    /**:
      ros__parameters:
        base_frame: world
    ```

    Then load this file when running the node:

    ```
    ros2 run myworkcell_core myworkcell_node __params:=my_params_file.yaml
    ```
   
### Launch file

1.  Currently, the `myworkcell_support` package only contains a launch file for starting the three
    required nodes. We will port it to a ROS2 python launch script.

1.  Start by creating a new file `workcell.launch.py` under the `launch/` directory.

1.  Modify the contents to add the required python imports and the required function which will
    return the launch configuration description:

    ``` py
    from launch import LaunchDescription
    import launch_ros.actions

    def generate_launch_description():
        print('Workcell is launching...')
    ```

1.  Each node in the XML launch file will correspond to a `Node` launch 'action' in the new launch
    description. The named arguments used in the action constructor should have clear meaning.
    You'll need one for the `fake_ar_publisher`:

    ```py
    fake_ar_pub_node = launch_ros.actions.Node(
        node_name='fake_ar_publisher_node',
        package='fake_ar_publisher',
        node_executable='fake_ar_publisher_node',
        output='screen',
        parameters=[{'x': -0.6},
                    {'y': 0.4},
                    {'z': 0.1},
                    {'camera_frame': 'camera_frame'}],
    )
    ```

    one for the `vision_node`:

    ```py
    vision_node = launch_ros.actions.Node(
        node_name='vision_node',
        package='myworkcell_core',
        node_executable='vision_node',
        output='screen',
    )
    ```

    and one for `myworkcell_node`:

    ```py
    myworkcell_node = launch_ros.actions.Node(
        node_name='myworkcell_node',
        package='myworkcell_core',
        node_executable='myworkcell_node',
        output='screen',
        parameters=[{'base_frame': 'world'}],
    )
    ```

    Notice the `parameters` argument must be set with a list of dictionaries. Another common
    argument not used here is `arguments`, set with a list of command line arguments that the
    executable will start with.

1.  Now bundle the three nodes in a `LaunchDescription` and return it from the function:

    ```py
    return LaunchDescription([fake_ar_pub_node, vision_node, myworkcell_node])
    ```

1.  Add an `install` call to `CMakesList.txt` to install the launch file:

    ``` 
    install(DIRECTORY launch
      DESTINATION share/${PROJECT_NAME}/
    )
    ```

1.  Modify `package.xml` to show the runtime dependency on the packages

    ``` xml
    <exec_depend>fake_ar_publisher</exec_depend>
    <exec_depend>myworkcell_core</exec_depend>
    ```

1.  After building the workspace test the launch file with:

    ```
    ros2 launch myworkcell_support workcell.launch.py
    ```

    Note that the file loaded by `ros2 launch` resides in the `install/` directory and will not
    update if you edit the version in the `src/` space. To avoid having to rebuild the workspace to
    see changes in the launch script, you can run `colcon build` with the `--symlink-install` option
    so the file in the install space is just a link back to the source space. This build option is
    also useful when developing ROS python code in general.
