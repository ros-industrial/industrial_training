# Topics and Messages
>In this exercise, we will explore the concept of ROS messages and topics.

## Motivation
The first type of ROS communication that we will explore is a one-way communication called messages which are sent over channels called topics. Typically one node publishes messages on a topic and another node subscribes to messages on that same topic. In this module we will create a subscriber node which subscribes to an existing publisher (topic/message).

## Reference Example

[Create a Subscriber](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

## Further Information and Resources

[Understanding Topics](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)

[Examining Publisher & Subscriber](http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber)

[Creating Messages and Services](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)

## Scan-N-Plan Application: Problem Statement
We now have a base ROS node and we want to build on this node. Now we want to create a subscriber within our node.

Your goal is to create your first ROS subscriber:
  1. First you will want to find out the message structure.
  1. You also want to determine the topic name.
  1. Last you can write the c++ code which serves as the subscriber.

## Scan-N-Plan Application: Guidance
### Add the fake_ar_publisher Package as a Dependency

1. Edit your package's `CMakeLists.txt` file (`~/catkin_ws/src/myworkcell_core/CMakeLists.txt`).  Make the following changes in the matching sections of the existing template file, by uncommenting and/or editing existing rules.

   1. Tell cmake to find the fake_ar_publisher package:

      ``` cmake
      ## Find catkin macros and libraries
      ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
      ## is used, also find other catkin packages
      find_package(catkin REQUIRED COMPONENTS 
        roscpp 
        fake_ar_publisher
      )
      ```

   1. Add The catkin runtime dependency for publisher.

      ``` cmake
      ## The catkin_package macro generates cmake config files for your package
      ## Declare things to be passed to dependent projects
      ## LIBRARIES: libraries you create in this project that dependent projects also need
      ## CATKIN_DEPENDS: catkin_packages dependent projects also need
      ## DEPENDS: system dependencies of this project that dependent projects also need
      catkin_package(
       #  INCLUDE_DIRS include
       #  LIBRARIES myworkcell_core
         CATKIN_DEPENDS 
           roscpp 
           fake_ar_publisher
      #  DEPENDS system_lib
      )
      ```

   1. Uncomment/edit the `add_dependencies` line __below__ your `add_executable` rule:
      
      ``` cmake
      add_dependencies(vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
      ```

1. add dependencies into your package's `package.xml`:

   ```xml
   <depend>fake_ar_publisher</depend>
   ```

1. `cd` into your catkin workspace

   ```
   cd ~/catkin_ws
   ```

1. Build your package and source the setup file to activate the changes in the current terminal.

   ```
   catkin build
   source ~/catkin_ws/devel/setup.bash
   ```

1. In a terminal, enter `rosmsg list`.  You will notice that, included in the list, is `fake_ar_publisher/ARMarker`.  If you want to see only the messages in a package, type `rosmsg package <package_name>`

1. Type `rosmsg show fake_ar_publisher/ARMarker`.  The terminal will return the types and names of the fields in the message.

   *Note that three fields under the `header` field are indented, indicating that these are members of the `std_msgs/Header` message type*

### Run a Publisher Node

1. In a terminal, type `rosrun fake_ar_publisher fake_ar_publisher_node`. You should see the program start up and begin publishing messages.

1. In another terminal, enter `rostopic list`.  You should see `/ar_pose_marker` among the topics listed. Entering `rostopic type /ar_pose_marker` will return the type of the message.

1. Enter `rostopic echo /ar_pose_marker`. The terminal will show the fields for each message as they come in, separated by a `---` line.  Press Ctrl+C to exit.

1. Enter `rqt_plot`.

   1. Once the window opens, type `/ar_pose_marker/pose/pose/position/x` in the "Topic:" field and click the "+" button. You should see the X value be plotted.

   1. Type `/ar_pose_marker/pose/pose/position/y` in the topic field, and click on the add button.  You will now see both the x and y values being graphed.

   1. Close the window

1. Leave the publisher node running for the next task.

## Create a Subscriber Node
1. Edit the `vision_node.cpp` file.

1. Include the message type as a header

   ``` c++
   #include <fake_ar_publisher/ARMarker.h>
   ```

1. Add the code that will be run when a message is received from the topic (the callback). 

   ``` c++
   class Localizer
   {
   public:
     Localizer(ros::NodeHandle& nh)
     {
         ar_sub_ = nh.subscribe<fake_ar_publisher::ARMarker>("ar_pose_marker", 1, 
         &Localizer::visionCallback, this);
     }

     void visionCallback(const fake_ar_publisher::ARMarkerConstPtr& msg)
     {
         last_msg_ = msg;
         ROS_INFO_STREAM(last_msg_->pose.pose);
     }
  
     ros::Subscriber ar_sub_;
     fake_ar_publisher::ARMarkerConstPtr last_msg_;
   };
   ```

1. Add the code that will connect the callback to the topic (within `main()`)

   ``` c++
   int main(int argc, char** argv)
   {
     ...
     // The Localizer class provides this node's ROS interfaces
     Localizer localizer(nh);

     ROS_INFO("Vision node starting");
     ...
   }
   ```
 
   * You can replace or leave the "Hello World" print... your choice!
   * These new lines must go below the `NodeHandle` declaration, so `nh` is actually defined.
   * Make sure to retain the `ros::spin()` call.  It will typically be the last line in your `main` routine.  Code after `ros::spin()` won't run until the node is shutting down.

1. Run `catkin build`, then `rosrun myworkcell_core vision_node`.

1. You should see the positions display from the publisher.

1. Press Ctrl+C on the publisher node.  The subscriber will stop displaying information.

1. Start the publisher node again. The subscriber will continue to print messages as the new program runs.

   * This is a key capability of ROS, to be able to restart individual nodes without affecting the overall system.

1. In a new terminal, type `rqt_graph`. You should see a window similar to the one below:

<p align="center"><img src=../../_static/simple_rqt_graph.png/></p>

   * The rectangles in the the window show the topics currently available on the system.
   * The ovals are ROS nodes.
   * Arrows leaving the node indicate the topics the node publishes, and arrows entering the node indicate the topics the node subscribes to.

