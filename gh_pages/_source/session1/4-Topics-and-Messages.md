# Topics and Messages
>In this exercise, we will explore the concept of ROS messages and topics.

## Motivation
The first type of ROS communication that we will explore is a one-way communication called messages which are sent over channels called topics. Typically one node publishes messages on a topic and another node subscribes to messages on that same topic. In this module we will create a subscriber node which subscribes to an existing publisher (topic/message).

## Reference Example

[Creating a Publisher and Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)

## Further Information and Resources

[Understanding Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

[Creating Messages and Services](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

## Scan-N-Plan Application: Problem Statement
We now have a base ROS node and we want to build on this node. Now we want to create a subscriber within our node.

Your goal is to create your first ROS subscriber:
  1. First you will want to find out the message structure.
  1. You also want to determine the topic name.
  1. Last you can write the c++ code which serves as the subscriber.

## Scan-N-Plan Application: Guidance
### Add the fake_ar_publisher Package as a Dependency

1. Edit your package's `CMakeLists.txt` file (`~/ros2_ws/src/myworkcell_core/CMakeLists.txt`).  Make the following changes in the file: 

   1. In the dependencies section, tell cmake to find the fake_ar_publisher package:

      ``` cmake
      find_package(fake_ar_publisher REQUIRED)
      ```

   1. Add the package to the list of dependencies of the _vision_node_ target:

      ``` cmake
      ament_target_dependencies(vision_node rclcpp fake_ar_publisher)
      ```

1. Add a dependency in your package's `package.xml`:

   ```xml
   <depend>fake_ar_publisher</depend>
   ```

### Inspect the Message Type

1. In a terminal, enter `ros2 interface package fake_ar_publisher`.  You will see all new ROS message types defined in that package. Only a single new type is defined: `fake_ar_publisher/msg/ARMarker`.

1. Type `ros2 interface show fake_ar_publisher/msg/ARMarker`.  The terminal will return the types and names of the fields in the message.

### Run a Publisher Node

1. In a terminal, type `ros2 run fake_ar_publisher fake_ar_publisher_node`. You should see the program start up and begin publishing messages.

1. In another terminal, enter `ros2 topic list`.  You should see `/ar_pose_marker` among the topics listed. Entering `ros2 topic type /ar_pose_marker` will return the type of the message.

1. Enter `ros2 topic echo /ar_pose_marker`. The terminal will show the fields for each message as they come in, separated by a `---` line.  Press Ctrl+C to exit.
  * It may look like the messages are static, but they are actually just continually publishing the same values.
  
1. Experiment with other `ros2 topic` commands to learn more about this topic:

   ```
   ros2 topic info /ar_pose_marker
   ros2 topic info -v /ar_pose_marker
   ros2 topic hz /ar_pose_marker
   ros2 topic bw /ar_pose_marker
   ```

1. In a new terminal, enter `ros2 run rqt_plot rqt_plot`.

   1. Once the window opens, type `/ar_pose_marker/pose/pose/position/x` in the "Topic:" field and click the "+" button. You should see the X value be plotted. If you can't see the X value, try changing the axes on your graph.

   1. Type `/ar_pose_marker/pose/pose/position/y` in the topic field, and click on the add button.  You will now see both the x and y values being graphed.

     * The plot doesn't look very interesting, since the simulated values are constant.  Use the plot tools to zoom out until you see both X & Y values plotted.  Can you use the plot data to estimate the XYZ position published by fake_ar_publisher?

   1. Close the window.

1. Leave the publisher node running for the next task.

## Create a Subscriber Node

We will now expand on the simple hello-world node created in _vision_node.cpp_ to subscribe to the _/ar_pose_marker_ topic.

1. Edit the `vision_node.cpp` file.

1. Include the message type as a header

   ``` c++
   #include <fake_ar_publisher/msg/ar_marker.hpp>
   ```

1. Add code above the `main` function that creates a new Node Class that subscribes to a topic of a type published by the _fake_ar_publisher_. The recommended ROS2 architecture is to create a class object for each Node.  This allows you to easily restructure your code later to assign either one or multiple nodes to a single process.

   ``` c++
   class Localizer : public rclcpp::Node
   {
   public:
     Localizer() : Node("vision_node"), last_msg_{nullptr}
     {
       ar_sub_ = this->create_subscription<fake_ar_publisher::msg::ARMarker>(
           "ar_pose_marker",
           rclcpp::QoS(1),
           std::bind(&Localizer::visionCallback, this, std::placeholders::_1));
     }

     void visionCallback(fake_ar_publisher::msg::ARMarker::SharedPtr msg)
     {
       last_msg_ = msg;
       RCLCPP_INFO(get_logger(), "Received pose: x=%f, y=%f, z=%f",
           msg->pose.pose.position.x,
           msg->pose.pose.position.y,
           msg->pose.pose.position.z);
     }

     rclcpp::Subscription<fake_ar_publisher::msg::ARMarker>::SharedPtr ar_sub_;
     fake_ar_publisher::msg::ARMarker::SharedPtr last_msg_;
   };
   ```

   The important lines to understand here are:

   1. `class Localizer : public rclcpp::Node`: This indicates any created `Localizer` object will be an independent ROS node.
   1. `ar_sub_ = this->create_subscription<...>(...)`: A subscriber object is created and stored in a member variable of the class.
   1. `<fake_ar_publisher::msg::ARMarker>` : The expected message type of the subscribed topic.
   1. `"ar_pose_marker",`: The topic name the subscription is associated with.
   1. `rclcpp::QoS(1),`: ROS2 has many options for controlling the _quality of service_ for communication between nodes, specified using the `QoS` type. Most options have defaults which are typically fine for normal use but a value is required to indicate the number of received messages to buffer, which is set as `1` here.
   1. `std::bind(&Localizer::visionCallback, this, std::placeholders::_1)`: This tells the subscriber object what function to call when new topic messages are received.  Note that the subscriber object is not passed a function pointer to the callback method directly, but instead uses `std::bind`. This is done because as a member function of a class, `visionCallback` must be bound to an instance of a `Localizer` object (i.e., the `this` pointer). Don't worry if the call to `std::bind` seems cryptic; the large majority of your subscriptions will be of this form and you can simply copy-paste the syntax.
   1. `void visionCallback(fake_ar_publisher::msg::ARMarker::SharedPtr msg)`: This is the callback function that will run anytime a new message is received on the topic. The callback for a subscription must have a signature of this form, with a single argument that contains the received message.  This particular callback function saves a copy of the last-received message and prints its value to the console.  This will help us for early debugging, but will later be replaced.

1. Add the code that will connect the callback to the topic (within `main()`)

   ``` c++
   int main(int argc, char** argv)
   {
     ...
     // The Localizer class provides this node's ROS interfaces
     auto node = std::make_shared<Localizer>();

     RCLCPP_INFO(node->get_logger(), "Vision node starting");
     ...
   }
   ```
 
   * You can replace or leave the "Hello World" print... your choice!
   * These new lines replace the original `rclcpp::Node` which was created directly. Remember the `Localizer` object itself is a ROS node.
   * Make sure to retain the `rclcpp::spin(node)` call. A node has to be spinning in order for any callbacks to actually execute. It will typically be the last line in your `main` routine.  Code after `rclcpp::spin()` won't run until the node is shutting down.

1. Run `colcon build`, then `ros2 run myworkcell_core vision_node`.

1. You should see the positions display from the publisher.

1. Press Ctrl+C on the publisher node.  The subscriber will stop displaying information.

1. Start the publisher node again. The subscriber will continue to print messages as the new program runs.

   * This is a key capability of ROS, to be able to restart individual nodes without affecting the overall system.

<!-- TODO: add the ROS2 equivalent of this back in

1. In a new terminal, type `ros2 run rqt_graph rqt_graph`. You should see a window similar to the one below:

<p align="center"><img src=../../_static/simple_rqt_graph.png/></p>

   * The rectangles in the the window show the topics currently available on the system.
   * The ovals are ROS nodes.
   * Arrows leaving the node indicate the topics the node publishes, and arrows entering the node indicate the topics the node subscribes to.
-->
