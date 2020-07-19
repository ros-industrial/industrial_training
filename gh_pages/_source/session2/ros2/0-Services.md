# Services
>In this exercise, we will create a custom service by defining a .srv file. Then we will write server and client nodes to utilize this service.

## Motivation
The first type of ROS communication that we explored was a one-way interaction called messages which are sent over channels called topics. Now we are going to explore a different communication type, which is a two-way interaction via a request from one node to another and a response from that node to the first. In this module we will create a service server (waits for request and comes up with response) and client (makes request for info then waits for response). 

## Reference Example

[Create a Service Server/Client](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Service-And-Client)

## Further Information and Resources

 * [Creating Messages & Services](https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces)
 * [Understanding Services](https://index.ros.org/doc/ros2/Tutorials/Services/Understanding-ROS2-Services)

## Scan-N-Plan Application: Problem Statement
We now have a base ROS node which is subscribing to some information and we want to build on this node. In addition we want this node to serve as a sub-function to another "main" node. The original vision node will now be responsible for subscribing to the AR information and responding to requests from the main workcell node. 

Your goal is to create a more intricate system of nodes:

1. Update the vision node to include a service server

1. Create a new node which will eventually run the Scan-N-Plan App
   * First, we'll create the new node (myworkcell_core) as a service client.  Later, we will expand from there 

## Scan-N-Plan Application: Guidance

### Create Service Definition

1. Similar to the message file located in the fake_ar_publisher package, we need to create a service file. The following is a generic structure of a service file:

   ```
   #request
   ---
   #response
   ```

1. Create a folder called `srv` inside your `myworkcell_core` package (at same level as the package's `src` folder)

   ```
   cd ~/ros2_ws/src/myworkcell_core
   mkdir srv
   ```
  
1. Create a file (gedit or QT) called `LocalizePart.srv` inside the `srv` folder.

1. Inside the file, define the service as outlined above with a request of type `string` named `base_frame` and a response that has a `geometry_msgs/Pose` field named `pose`, and a boolean field named `success`:

   ```
   #request
   string base_frame
   ---
   #response
   geometry_msgs/Pose pose
   bool success
   ```

   * _An explicit member of the response that indicates success or failure is usually good practice with services, often along with a string member that can provide error information when a failure does happen._

1. Edit the package's `CMakeLists.txt` and `package.xml` to add dependencies on key packages:

   * The `rosidl_default_generators` package is required to build C++ code from the .srv file created in the previous step
   * The `rosidl_default_runtime` package provides runtime dependencies for new messages
   * `geometry_msgs` provides the `Pose` message type used in our service definition

   1. Edit the package's `CMakeLists.txt` file to add the new **build-time** dependencies alongside the existing `find_package` rules:
      
      ``` cmake
      find_package(geometry_msgs REQUIRED)
      find_package(rosidl_default_generators REQUIRED)
      ```

   1. Edit the `package.xml` file to add the appropriate build/run dependencies:
     
      ``` xml
      <build_depend>rosidl_default_generators</build_depend>
      <exec_depend>rosidl_default_runtime</exec_depend>
      <depend>geometry_msgs</depend>
      ```

    1. Also add a special tag that tells the ROS build system this package defines ROS interfaces.

       ```xml
       <member_of_group>rosidl_interface_packages</member_of_group>
       ```

       This is generally placed with the package metadata, before the dependencies.


1. Edit the package's `CMakeLists.txt` to add rules to generate the new service files:

   1. The following CMake function is invoked with the service file you created and creates the target to generate the interface-specific code:


      ``` cmake
      rosidl_generate_interfaces(${PROJECT_NAME}
        "srv/LocalizePart.srv"
        DEPENDENCIES geometry_msgs
      )
      ```

   1. Add the following line after the call to `ament_target_dependencies` for the the _vision_node_. This makes the _vision_node_ target depend on the generated interface target so that the generated code can be used in the node:
     
      ``` cmake
      rosidl_target_interfaces(vision_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
      ```

1. NOW! you have a service defined in you package and you can attempt to _build_ the code to generate the service:
   
   ```
   colcon build
   ```

### Service Server

1. Edit `vision_node.cpp`; remember that the [ros documentation](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Service-And-Client) is a resource.

1. Add the header for the service we just created

   ``` c++
    #include <myworkcell_core/srv/localize_part.hpp>
   ```

1. Add a member variable named `server_` near the other `Localizer` class member variables:

   ``` c++
   rclcpp::Service<myworkcell_core::srv::LocalizePart>::SharedPtr server_;
   ```

1. In the `Localizer` class constructor, advertise your service to the ROS master:

   ``` c++
   server_ = this->create_service<myworkcell_core::srv::LocalizePart>(
     "localize_part",
     std::bind(&Localizer::localizePart, this, std::placeholders::_1, std::placeholders::_2));
   ```

   _Note the use of `std::bind` again to create a function object that the service will use a callback. The use of two `std::placeholders` arguments indicates the callback will have two arguments when it is called._

1. The `create_service` command above referenced a service callback named `localizePart`. Create an empty function with this name in the `Localizer` class. Remember that your request and response types were defined in the `LocalizePart.srv` file. The arguments to the boolean function are the request and response type, with the general structure of `Package::ServiceName::Request` or `Package::ServiceName::Response`. 

   ``` c++
   void localizePart(myworkcell_core::srv::LocalizePart::Request::SharedPtr req,
                     myworkcell_core::srv::LocalizePart::Response::SharedPtr res)
   {

   }
   ```

1. Now add code to the `localizePart` callback function to fill in the Service Response. Eventually, this callback will transform the pose received from the `fake_ar_publisher` (in `visionCallback`) into the frame specifed in the Service Request.  For now, we will skip the frame-transform, and just pass through the data received from `fake_ar_publisher`.  Copy the pose measurement received from `fake_ar_publisher` (saved to `last_msg_`) directly to the Service Response. 

   ``` c++
   void localizePart(myworkcell_core::srv::LocalizePart::Request::SharedPtr req,
                     myworkcell_core::srv::LocalizePart::Response::SharedPtr res)
   {
     // Read last message
     fake_ar_publisher::msg::ARMarker::SharedPtr p = last_msg_;

     if (! p)
     {
       RCLCPP_ERROR(this->get_logger(), "no data");
       res->success = false;
       return;
     }

     res->pose = p->pose.pose;
     res->success = true;
   }
   ```

1. You should comment out the `RCLCPP_INFO` call in your `visionCallback` function, to avoid cluttering the screen with useless info.

1. Build the updated `vision_node`, to make sure there are no compile errors.

### Service Client

1. Create a new node (inside the same `myworkcell_core` package), named `myworkcell_node.cpp`.  This will eventually be our main "application node", that controls the sequence of actions in our Scan & Plan task.  The first action we'll implement is to request the position of the AR target from the Vision Node's `LocalizePart` service we created above.

1. Be sure to include the standard ros header as well as the header for the `LocalizePart` service:

   ``` c++
   #include <rclcpp/rclcpp.hpp>
   #include <myworkcell_core/srv/localize_part.hpp>
   ```

1. Create a standard C++ main function, with typical ROS initialization:

   ``` c++
   int main(int argc, char **argv)
   {
     // This must be called before anything else ROS-related
     rclcpp::init(argc, argv);

     rclcpp::shutdown();
     return 0;
   }
   ```

1. We will be using a cpp class "ScanNPlan" to contain the functionality of the myworkcell_node.  Create a skeleton structure of this class, with an empty constructor and a private area for some internal/private variables.

   ``` c++
   class ScanNPlan : public rclcpp::Node
   {
   public:
     ScanNPlan() : Node("scan_n_plan")
     {

     }

   private:
     // Planning components

   };
   ```

1. Within your new ScanNPlan class, define a ROS Client as a private member variable of the class.  Initialize the Client in the ScanNPlan constructor, using the same service name as defined earlier ("localize_part"). Create a void function within the ScanNPlan class named `start`, with no arguments. This will contain most of our application workflow.  For now, this function will call the `LocalizePart` service and print the response. 

   ``` c++
   class ScanNPlan : public rclcpp::Node
   {
   public:
     ScanNPlan() : Node("scan_n_plan")
     {
       vision_client_ = this->create_client<myworkcell_core::srv::LocalizePart>("localize_part");
     }

     void start()
     {
       RCLCPP_INFO(get_logger(), "Attempting to localize part");

       // Create a request for the LocalizePart service call
       auto request = std::make_shared<myworkcell_core::srv::LocalizePart::Request>();

       auto future = vision_client_->async_send_request(request);
     }

   private:
     // Planning components
     rclcpp::Client<myworkcell_core::srv::LocalizePart>::SharedPtr vision_client_;
   };
   ```

   The `async_send_request` function is used on the `vision_client_` object to initiate the service call. Notice that it doesn't return a response object, but instead, something called a `future`. This is because the function immediately returns after sending the request and does not wait for a response from the server. It is our responsibility to wait for the response to arrive, which will be known by the future object becoming 'complete'.


1. Now add the code that will wait until the service response arrives and then prints the response:

   ``` c++
   void start()
   {
     RCLCPP_INFO(get_logger(), "Attempting to localize part");

     // Create a request for the LocalizePart service call
     auto request = std::make_shared<myworkcell_core::srv::LocalizePart::Request>();

     auto future = vision_client_->async_send_request(request);

     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::executor::FutureReturnCode::SUCCESS)
     {
       RCLCPP_ERROR(this->get_logger(), "Failed to receive LocalizePart service response");
       return;
     }

     auto response = future.get();
     if (! response->success)
     {
       RCLCPP_ERROR(this->get_logger(), "LocalizePart service failed");
       return;
     }

     RCLCPP_INFO(this->get_logger(), "Part Localized: x: %f, y: %f, z: %f",
         response->pose.position.x,
         response->pose.position.y,
         response->pose.position.z);
   }
   ```

   * Note that the waiting function is a form of spinning, the same as the `rclcpp::spin` function used in the _vision_node_. This is because under the hood, the service response is arriving on a topic and a callback needs to run inside the `Client` class to received the response message.
   * _Important_: Spinning a node is exclusive, so you can't use the `spin_until_future_complete` function if the main function is already spinning the node. This would be the case if you were trying to call a service from inside a callback, for example. The ROS2 defaults are that only one callback can run at a time, so this scenario leads to deadlock because if you try to wait for the response inside a callback, the response callback will never be run. There are ways around this limitation but it is beyond the scope of this material.
   * An alternative to waiting for the future is to provide a callback along with the service request which will run when the response arrives. For some program designs this might make more sense and this technique can be seen in the official ROS2 examples.

1. Now back in `myworkcell_node`'s `main` function, instantiate an object of the `ScanNPlan` class and call the object's `start` function.
 
   ``` c++
   // Create the ScanNPlan node
   auto app = std::make_shared<ScanNPlan>();
   app->start();
   ```

1. Edit the package's `CMakeLists.txt` to build the new node (executable), with its associated dependencies.  Add the following rules to the appropriate sections, directly under the matching rules for `vision_node`:

   ``` cmake
   add_executable(myworkcell_node src/myworkcell_node.cpp)
   ament_target_dependencies(myworkcell_node rclcpp)
   rosidl_target_interfaces(myworkcell_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
   ``` 

   Also add `myworkcell_node` to the existing call to `install` along with `vision_node`:

   ``` cmake
   install(TARGETS vision_node myworkcell_node
       ...
   )
   ```

5. Build the nodes to check for any compile-time errors:

   ``` bash
   colcon build
   ```

### Use New Service

1. Enter each of these commands in their own terminal:

   ```
   ros2 run fake_ar_publisher fake_ar_publisher_node
   ros2 run myworkcell_core vision_node
   ros2 run myworkcell_core myworkcell_node
   ```
