# Services
>In this exercise, we will create a custom service by defining a .srv file. Then we will write server and client nodes to utilize this service.

## Motivation
The first type of ROS communication that we explored was a one-way interaction called messages which are sent over channels called topics. Now we are going to explore a different communication type, which is a two-way interaction via a request from one node to another and a response from that node to the first. In this module we will create a service server (waits for request and comes up with response) and client (makes request for info then waits for response). 

## Reference Example

[Create a Service Server/Client](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)

## Further Information and Resources

 * [Creating Messages & Services](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
 * [Understanding Services & Params](http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams)
 * [Examining Service Client](http://wiki.ros.org/ROS/Tutorials/ExaminingServiceClient)

## Scan-N-Plan Application: Problem Statement
We now have a base ROS node which is subscribing to some information and we want to build on this node. In addition we want this node to serve as a sub-function to another "main" node. The original vision node will now be responsible for subscribing to the AR information and responding to requests from the main workcell node. 

Your goal is to create a more intricate system of nodes:

1. Update the vision node to include a service server

2. Create a new node which will eventually run the Scan-N-Plan App
   * First, we'll create the new node (myworkcell_core) as a service client.  Later, we will expand from there 

## Scan-N-Plan Application: Guidance

### Create Service Definition

1. Similar to the message file located in the fake_ar_publisher package, we need to create a service file. The following is a generic structure of a service file:

   ```
   #request
   ---
   #response
   ```

2. Create a folder called `srv` inside your `myworkcell_core` package (at same level as the package's `src` folder)

   ```
   cd ~/catkin_ws/src/myworkcell_core
   mkdir srv
   ```
  
3. Create a file (gedit or QT) called `LocalizePart.srv` inside the `srv` folder.

4. Inside the file, define the service as outlined above with a request of type `string` named `base_frame` and a response of type `geometry_msgs/Pose` named `pose`:

   ```
   #request
   string base_frame
   ---
   #response
   geometry_msgs/Pose pose
   ```

5. Edit the package's `CMakeLists.txt` and `package.xml` to add dependencies on key packages:

   * `message_generation` is required to build C++ code from the .srv file created in the previous step
   * `message_runtime` provides runtime dependencies for new messages
   * `geometry_msgs` provides the `Pose` message type used in our service definition

   1. Edit the package's `CMakeLists.txt` file to add the new **build-time** dependencies to the existing `find_package` rule:
      
      ``` cmake
      find_package(catkin REQUIRED COMPONENTS
        roscpp
        fake_ar_publisher
        geometry_msgs
        message_generation
      )
      ```

   2. Also in `CMakeLists.txt`, add the new **run-time** dependencies to the existing `catkin_package` rule:
      
      ``` cmake
      catkin_package(
      #  INCLUDE_DIRS include
      #  LIBRARIES myworkcell_node
        CATKIN_DEPENDS
          roscpp
          fake_ar_publisher
          message_runtime
          geometry_msgs
      #  DEPENDS system_lib
      )
      ```

   3. Edit the `package.xml` file to add the appropriate build/run dependencies:
     
      ``` xml
      <build_depend>message_generation</build_depend>
      <exec_depend>message_runtime</exec_depend>
      <build_depend>geometry_msgs</build_depend>
      <exec_depend>geometry_msgs</exec_depend>
      ```

6. Edit the package's `CMakeLists.txt` to add rules to generate the new service files:

   1. Uncomment/edit the following `CMakeLists.txt` rule to reference the `LocalizePart` service we defined earlier:

      ``` cmake
      ## Generate services in the 'srv' folder
      add_service_files(
         FILES
         LocalizePart.srv
      )
      ```

   2. Uncomment/edit the following `CMakeLists.txt` rule to enable generation of messages and services:
     
      ``` cmake
      ## Generate added messages and services with any dependencies listed here
      generate_messages(
         DEPENDENCIES
         geometry_msgs
      )
      ```

7. NOW! you have a service defined in you package and you can attempt to _Build_ the code to generate the service:
   
   ```
   catkin build
   ```
   _Note: (or use Qt!)_

### Service Server

1. Edit `vision_node.cpp`; remember that the [ros wiki](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29) is a resource.

2. Add the header for the service we just created

   ``` c++
    #include <myworkcell_core/LocalizePart.h>
   ```

3. Add a member variable (type: `ServiceServer`, name: `server_`), near the other `Localizer` class member variables:

   ``` c++
   ros::ServiceServer server_;
   ```

4. In the `Localizer` class constructor, advertise your service to the ROS master:

   ``` c++
   server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this);
   ```

5. The `advertiseService` command above referenced a service callback named `localizePart`. Create an empty boolean function with this name in the `Localizer` class. Remember that your request and response types were defined in the `LocalizePart.srv` file. The arguments to the boolean function are the request and response type, with the general structure of `Package::ServiceName::Request` or `Package::ServiceName::Response`. 

   ``` c++
   bool localizePart(myworkcell_core::LocalizePart::Request& req,
                     myworkcell_core::LocalizePart::Response& res)
   {

   }
   ```

6. Now add code to the `localizePart` callback function to fill in the Service Response. Eventually, this callback will transform the pose received from the `fake_ar_publisher` (in `visionCallback`) into the frame specifed in the Service Request.  For now, we will skip the frame-transform, and just pass through the data received from `fake_ar_publisher`.  Copy the pose measurement received from `fake_ar_publisher` (saved to `last_msg_`) directly to the Service Response. 

   ``` c++
   bool localizePart(myworkcell_core::LocalizePart::Request& req,
                     myworkcell_core::LocalizePart::Response& res)
   {
     // Read last message
     fake_ar_publisher::ARMarkerConstPtr p = last_msg_;  
     if (!p) return false;

     res.pose = p->pose.pose;
     return true;
   }
   ```

7. You should comment out the `ROS_INFO_STREAM` call in your `visionCallback` function, to avoid cluttering the screen with useless info.

8. Build the updated `vision_node`, to make sure there are no compile errors.

### Service Client

1. Create a new node (inside the same `myworkcell_core` package), named `myworkcell_node.cpp`.  This will eventually be our main "application node", that controls the sequence of actions in our Scan & Plan task.  The first action we'll implement is to request the position of the AR target from the Vision Node's `LocalizePart` service we created above.

1. Be sure to include the standard ros header as well as the header for the `LocalizePart` service:

   ``` c++
   #include <ros/ros.h>
   #include <myworkcell_core/LocalizePart.h>
   ```

1. Create a standard C++ main function, with typical ROS node initialization:

   ``` c++
   int main(int argc, char **argv)
   {
     ros::init(argc, argv, "myworkcell_node");
     ros::NodeHandle nh;

     ROS_INFO("ScanNPlan node has been initialized");

     ros::spin();
   }
   ```

1. We will be using a cpp class "ScanNPlan" to contain most functionality of the myworkcell_node.  Create a skeleton structure of this class, with an empty constructor and a private area for some internal/private variables.

   ``` c++
   class ScanNPlan
   {
   public:
     ScanNPlan(ros::NodeHandle& nh)
     {

     }

   private:
     // Planning components

   };
   ```

1. Within your new ScanNPlan class, define a ROS ServiceClient as a private member variable of the class.  Initialize the ServiceClient in the ScanNPlan constructor, using the same service name as defined earlier ("localize_part"). Create a void function within the ScanNPlan class named `start`, with no arguments. This will contain most of our application workflow.  For now, this function will call the `LocalizePart` service and print the response. 

   ``` c++
   class ScanNPlan
   {
   public:
     ScanNPlan(ros::NodeHandle& nh)
     {
       vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
     }

     void start()
     {
       ROS_INFO("Attempting to localize part");
       // Localize the part
       myworkcell_core::LocalizePart srv;
       if (!vision_client_.call(srv))
       {
         ROS_ERROR("Could not localize part");
         return;
       }
       ROS_INFO_STREAM("part localized: " << srv.response);
     }

   private:
     // Planning components
     ros::ServiceClient vision_client_;
   };
   ```

1. Now back in `myworkcell_node`'s `main` function, instantiate an object of the `ScanNPlan` class and call the object's `start` function.
 
   ``` c++
   ScanNPlan app(nh);

   ros::Duration(.5).sleep();  // wait for the class to initialize
   app.start();
   ```
1. Edit the package's `CMakeLists.txt` to build the new node (executable), with its associated dependencies.  Add the following rules to the appropriate sections, directly under the matching rules for `vision_node`:

   ``` cmake
   add_executable(myworkcell_node src/myworkcell_node.cpp)

   add_dependencies(myworkcell_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

   target_link_libraries(myworkcell_node ${catkin_LIBRARIES})
   ``` 

5. Build the nodes to check for any compile-time errors:

   ``` bash
   catkin build
   ```
   _Note: (or use Qt!)_

### Use New Service

1. Enter each of these commands in their own terminal:

   ```
   roscore
   rosrun fake_ar_publisher fake_ar_publisher_node
   rosrun myworkcell_core vision_node
   rosrun myworkcell_core myworkcell_node
   ```
