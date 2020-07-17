# Parameters
>In this exercise, we will look at ROS Parameters for configuring nodes

## Motivation
By this point in these tutorials (or your career), you've probably typed the words `int main(int argc, char** argv)` more times than you can count. The arguments to `main` are the means by which a system outside scope and understanding of your program can configure your program to do a particular task. These are _command line parameters_.

The ROS ecosystem has an analogous system for configuring nodes. It's a key-value storage model that is associated with each node individually. It's best used to pass configuration parameters to nodes at run-time (e.g. to identify which camera a node should subscribe to), but it can be used for much more complicated items.

## Reference Example
[Using Parameters](https://index.ros.org/doc/ros2/Tutorials/Using-Parameters-In-A-Class-CPP)

## Further Information and Resource
[Understanding ROS2 parameters](https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters)

## Scan-N-Plan Application: Problem Statement
In previous exercises, we added a service with the following definition:
  ```
  # request
  string base_frame
  ---
  # response
  geometry_msgs/Pose pose
  bool success
  ```

So far we haven't used the request field, `base_frame`, for anything. In this exercise we'll use ROS parameters to set this field. You will need to:

1. Declare inside the class that the Node uses a `base_frame` parameter of string type.
1. Load the parameter `base_frame` and store it in a local string object.

   * If no parameter is provided, default to the parameter to `"world"`.

1. When making the service call to the `vision_node`, use this parameter to fill out the `Request::base_frame` field.
1. Add a `parameters` arguments to your launch file to initialize the new value.

## Scan-N-Plan Application: Guidance

1. Open up `myworkcell_node.cpp` for editing.

1. Inside the `ScanNPlan` class constructor, add a call to `declare_parameter`:

   ``` c++
   this->declare_parameter("base_frame", "world");
   ```

   * The first argument is the parameter name, the second is the default value if the parameter is not set.
   * The parameter type is fixed from the type of the default value. You may safely assume any value you get when accessing the _base_frame_ parameter is a string.

1. In the main function, after creating the node but before calling `start`, create a temporary string object, `std::string base_frame;`, and then use `get_parameter` to load the parameter `"base_frame"`.

   ``` c++
   std::string base_frame;
   app->get_parameter("base_frame", base_frame);
   ```

   * Note we didn't do any checking that the parameter exists or has the right type. This because the `declare_parameter` function provides these guarantees.
   * The `declare_parameter` function actually returns the parameter value as well, so if you want to declare and get a parameter in the same place, you can do it with one line.

1. Add an argument to your `myworkcell_node` "start" function of a string named `base_frame`, and assign the value from the argument into the service request. Make sure to update the `app->start` call in your `main()` routine to pass through the `base_frame` value you obtained.

   ``` c++
   void start(const std::string& base_frame)
   {
     ...
     request->base_frame = base_frame;
     RCLCPP_INFO_STREAM("Requesting pose in base frame: " << base_frame);
     ...
   }

   int main(...)
   {
     ...
     app->start(base_frame);
     ...
   }
   ```

5. Now we'll add `myworkcell_node` to the existing `workcell.launch.py` file, so we can set the `base_frame` parameter from a launch file.  We'd like the `vision_node` to return the position of the target relative to the world frame, for motion-planning purposes.  Even though that's the default value, we'll specify it in the launch-file anyway:

   ``` py
   launch_ros.actions.Node(
       node_name='myworkcell_node',
       package='myworkcell_core',
       node_executable='myworkcell_node',
       output='screen',
       parameters=[{'base_frame': 'world'}],
   )
   ```

   * Note that the `parameters` arguments provides the names and values to use as a _list of dictionaries_.

6. Try it out by running the system.

   ```
   colcon build
   ros2 launch myworkcell_support workcell.launch
   ```

    * Press _Ctrl+C_ to kill the running nodes
    * Edit the launch file to change the base_frame parameter value (e.g. to "test2")
    * Re-launch workcell.launch.py, and observe that the "request frame" has changed
         - The response frame doesn't change, because we haven't updated vision_node (yet) to handle the request frame.  Vision_node always returns the same frame (for now).
    * Set the base_frame back to "world"
