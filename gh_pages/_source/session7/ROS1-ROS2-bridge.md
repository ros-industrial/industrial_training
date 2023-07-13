# ROS1-ROS2 Bridge Demo

## Introduction

This is a system integration exercise to demonstrate operation of the ROS1-ROS2 topic and service
bridge. Using the bridge does not require new techniques when developing either ROS1 or ROS2
software, so much of the required code is provided to you in a template workspace (`~/industrial_training/exercises/7.2/template_ws/src`). Instead, we will focus on developing code that interfaces with the `ros1_bridge` package.

In this exercise, the ROS1 `ARMarker` message, which is the goal destination for the robot, must be communicated to the ROS2 application. Then, the ROS2 application needs to use service clients to call a planner that only
exists as a ROS1 package. Specifically, this exercise calls the Descartes motion planner
as seen in Exercise 4.1. On the ROS1 side, service servers are provided to generate motion plans and to
execute them. In order to use these custom services, the bridge needs to be compiled from source 
with the message definitions in the build environment, which is the bulk of the complexity in this 
exercise.

## Building the Demo

This exercise uses the ROS1 bridge to call ROS1 nodes from ROS2 nodes and therefore the build
procedure is somewhat involved.

### Create a ROS1 workspace from Exercise 4.1

1.  Open a new terminal. Create a catkin workspace for the exercise 4.1 ROS1 packages and dependencies.
    ```
    mkdir -p ~/catkin_ws_to_bridge/src
    ```

1.  Copy Exercise 4.1 in the training repo.
    ```
    cp -r ~/industrial_training/exercises/4.1/ros1/src ~/catkin_ws_to_bridge/src/demo
    ```

1.  Clone additional dependencies.
    ```
    cd ~/catkin_ws_to_bridge/src
    git clone --branch melodic-devel https://github.com/ros-industrial-consortium/descartes.git
    git clone --branch kinetic-devel https://github.com/ros-industrial/universal_robot.git
    git clone --branch master https://github.com/ros-industrial/fake_ar_publisher.git
    ```

1.  Source and build exercise from the top workspace directory.
    ```
    source /opt/ros/noetic/setup.bash
    cd ~/catkin_ws_to_bridge
    catkin build    
    ```

    From the above steps, the ROS1 catkin workspace is being provided without requiring edits. Take note of the
    critical components that will be communicated over the bridge. 
    
    There is an `ARMarker` topic message (directory `~/catkin_ws_to_bridge/src/fake_ar_publisher/msg`) published by the node within `fake_ar_publisher.cpp` (directory `~/catkin_ws_to_bridge/src/fake_ar_publisher/src`). 
    
    Additionally, there are service messages (directory `~/catkin_ws_to_bridge/src/demo/myworkcell_core/srv`), and the service servers will be adverstized by nodes (directory `~/catkin_ws_to_bridge/src/demo/myworkcell_core/src/`).

    > Service message `LocalizePart`      advertised by node within `vision_node.cpp`
    
    > Service message `PlanCartesianPath` advertised by node within `descartes_node.cpp`
    
    > Service message `ExecuteTrajectory` advertised by node within `move_robot.cpp`
    
    > Service message `MoveToPose`        advertised by node within `move_robot.cpp`


### Create the ROS2 workspace

1.  In a new terminal, create a colcon workspace for the exercise 7.2 ROS1 packages and dependencies. We will copy a
template with part of the exercise created for you.
    ```
    mkdir -p ~/colcon_ws/src
    cd ~/colcon_ws/src
    ```
    ```
    cp -r ~/industrial_training/exercises/7.2/template_ws/src ~/colcon_ws/src/demo
    ```

1.  Open the blank file located at the following directory:
    ```
    gedit ~/colcon_ws/src/demo/myworkcell_msgs/message_mappings.yaml
    ```
    Add the message mapping parameters to the blank file. This will map four services that we plan on using in our ROS2 node (`~/colcon_ws/src/demo/myworkcell_core/src/myworkcell_core.cpp`). These are necessary to call the planner in ROS1 and command the robot to execute the trajectory. Additionally, we will map one publisher/subscriber message called `ARMarker`,
    but the ROS2 C++ code will not require this message. We will use it in the terminal in the later section called "Communicate a Publisher/subscriber topic over the bridge." The final contents of the YAML file should be as follows:
    ```
    -
      ros1_package_name: 'myworkcell_core'
      ros1_service_name: 'PlanCartesianPath'
      ros2_package_name: 'myworkcell_msgs'
      ros2_service_name: 'PlanCartesianPath'
    -
      ros1_package_name: 'myworkcell_core'
      ros1_service_name: 'MoveToPose'
      ros2_package_name: 'myworkcell_msgs'
      ros2_service_name: 'MoveToPose'
    -
      ros1_package_name: 'myworkcell_core'
      ros1_service_name: 'ExecuteTrajectory'
      ros2_package_name: 'myworkcell_msgs'
      ros2_service_name: 'ExecuteTrajectory'
    -
      ros1_package_name: 'myworkcell_core'
      ros1_service_name: 'LocalizePart'
      ros2_package_name: 'myworkcell_msgs'
      ros2_service_name: 'LocalizePart' 
    -
      ros1_package_name: 'fake_ar_publisher'
      ros1_message_name: 'ARMarker'
      ros2_package_name: 'myworkcell_msgs'
      ros2_message_name: 'ARMarker'
    ```

    Save and close the file `~/colcon_ws/src/demo/myworkcell_msgs/message_mappings.yaml`.

1.  Open the ScanNPlan node file file at the following directory:
    ```
    gedit ~/colcon_ws/src/demo/myworkcell_core/src/myworkcell_core.cpp
    ```
    
    Add the definitions and declarations for the services that will be communicated over the bridge when calling the ROS1 planner.

    ```C++
    class ScanNPlan: public rclcpp::Node
    {
    public:
      ScanNPlan(const rclcpp::NodeOptions & options):
      rclcpp::Node(NODE_NAME, options)
      {
        //***FILL CODE HERE IN CONSTRUCTOR FOR SERVICES
        vision_client_ = this->create_client<myworkcell_msgs::srv::LocalizePart>("localize_part");
        cartesian_client_ = this->create_client<myworkcell_msgs::srv::PlanCartesianPath>("plan_path");
        move_client_ = this->create_client<myworkcell_msgs::srv::MoveToPose>("move_to_pose");
        exec_traj_client_ = this->create_client<myworkcell_msgs::srv::ExecuteTrajectory>("execute_trajectory");
      }
  
      ~ScanNPlan()
      {
      
      }
  
      bool start(const std::string& base_frame)
      {
        //***FILL CODE HERE TO CALL THE ROS SERVICES OVER THE BRIDGE
      }
  
      //***FILL CODE HERE TO DECLARE SERVICES
      rclcpp::Client<myworkcell_msgs::srv::LocalizePart>::SharedPtr vision_client_;
      rclcpp::Client<myworkcell_msgs::srv::PlanCartesianPath>::SharedPtr cartesian_client_;
      rclcpp::Client<myworkcell_msgs::srv::MoveToPose>::SharedPtr move_client_;
      rclcpp::Client<myworkcell_msgs::srv::ExecuteTrajectory>::SharedPtr exec_traj_client_;
    };
    ```

1.  Add a `start()` method that will call the service servers from ROS1. The final start method should appear
as follows to use all four services we previously mapped.

    ```C++
      bool start(const std::string& base_frame)
      {
        //***FILL CODE HERE TO CALL THE ROS SERVICES OVER THE BRIDGE
        using namespace myworkcell_msgs::srv;
  
        // waiting for services
        std::vector<rclcpp::ClientBase* > clients = {vision_client_.get(),
                                                     cartesian_client_.get() ,
                                                     move_client_.get(),
                                                     exec_traj_client_.get()};
        if(std::all_of(clients.begin(), clients.end(),[](rclcpp::ClientBase* c){
          std::cout<<"Waiting for client "<< c->get_service_name()<<std::endl;
          return c->wait_for_service(std::chrono::seconds(WAIT_SERVICE_PERIOD));
        }))
        {
          std::cout<<"Found all services"<<std::endl;
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "The service was not found");
          return false;
        }
    
        // sending localization request
        LocalizePart::Request::SharedPtr req = std::make_shared<LocalizePart::Request>();
        req->base_frame = base_frame;
        std::cout<<"Requesting pose in base frame: "<< base_frame << std::endl;
        //RCLCPP_INFO(this->get_logger(),"Requesting pose in base frame: %s", base_frame.c_str()); // TODO: enable this when     printing to console works again
    
        std::shared_future<LocalizePart::Response::SharedPtr> result_future = vision_client_->async_send_request(req);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),result_future) !=     rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_ERROR(this->get_logger(), "Localize service call failed");
          return false;
        }
    
        if(!result_future.get()->succeeded)
        {
          RCLCPP_ERROR(this->get_logger(), "Could not localize part");
          return false;
        }
    
        // getting response
        LocalizePart::Response::SharedPtr res = result_future.get();
        std::cout<<"Part localized"<<std::endl;
        geometry_msgs::msg::Pose move_target = res->pose;
    
        // moving to pose
        MoveToPose::Request::SharedPtr move_req = std::make_shared<MoveToPose::Request>();
        move_req->pose.header.frame_id = base_frame;
        move_req->pose.pose = move_target;
        std::shared_future<MoveToPose_Response::SharedPtr> move_future = move_client_->async_send_request(move_req);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),move_future) !=     rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_ERROR(this->get_logger(),"Failed to move to target");
          return false;
        }
    
        // planning cartesian path
        PlanCartesianPath_Request::SharedPtr plan_req = std::make_shared<PlanCartesianPath::Request>();
        plan_req->pose = move_target;
        std::cout<<"Planning trajectory"<<std::endl;
        std::shared_future<PlanCartesianPath_Response::SharedPtr> plan_future = cartesian_client_->async_send_request(plan_req);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),plan_future) !=     rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_ERROR(this->get_logger(),"Could not plan for path");
          return false;
        }
    
        // executing
        ExecuteTrajectory::Request::SharedPtr exec_req = std::make_shared<ExecuteTrajectory::Request>();
        exec_req->trajectory = plan_future.get()->trajectory;
        std::cout<<"Executing trajectory"<<std::endl;
        std::shared_future<ExecuteTrajectory::Response::SharedPtr> exec_future = exec_traj_client_->async_send_request(exec_req);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), exec_future) !=     rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_ERROR(this->get_logger(),"Failed to execute trajectory");
          return false;
        }
        std::cout<<"Trajectory execution complete"<<std::endl;
        return true;
      }
    ```

1.  Add a main method that starts ROS2 and calls the `start()` method after instantiation of the `ScanNPlan` class.
    ```C++
    int main(int argc, char** argv)
    {
      //***FILL CODE HERE TO START ROS2, INSTANTIATE THE ScanNPlan CLASS, AND CALL THE start() METHOD
      rclcpp::init(argc,argv);
      std::shared_ptr<ScanNPlan> app = std::make_shared<ScanNPlan>(rclcpp::NodeOptions());
    
      // getting parameter
      rclcpp::ParameterValue base_frame_param = app->declare_parameter("base_frame");
      if(base_frame_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
      {
        RCLCPP_ERROR(app->get_logger(),"Parameter not found");
        return -1;
      }
      std::string base_frame = base_frame_param.get<std::string>();
      std::cout<<"Got base_frame parameter " << base_frame << std::endl;
    
      if(!app->start(base_frame))
      {
        rclcpp::shutdown();
        return -1;
      }
    
      rclcpp::shutdown();
      return 0;
    }
    ```
    Save and close the file `~/colcon_ws/src/demo/myworkcell_core/src/myworkcell_core.cpp`.

1.  Build the ROS2 workspace.
    ```
    cd ~/colcon_ws
    source /opt/ros/foxy/setup.bash
    colcon build
    ```

### Build the ROS1 Bridge 

If you have `ros2_control` installed, you may encounter a known error while building the `ros1_bridge` ([#329](https://github.com/ros2/ros1_bridge/issues/329)). This can be avoided by purging `controller_manager_msgs` from the `foxy` installation. 

```
sudo apt purge ros-foxy-controller-manager-msgs --autoremove
```

At the end of Exercise 7.2, the last instruction will remind you to reinstall `ros2_control` so the package can be used during other demos and exercises. 


1.  Open a new terminal. Create the ros1_bridge workspace. We build the bridge in a separate workspace because it needs
    to see both ROS1 and ROS2 packages in its environment, and we want to make sure our application
    workspaces only see the packages from the distribution they are in.
    ```
    mkdir -p ~/ros1_bridge_ws/src
    cd ~/ros1_bridge_ws/src
    ```

1.  Clone the ROS1 bridge into your ROS2 workspace, selecting the branch that matches your ROS
    release.
    ```
    git clone -b foxy https://github.com/ros2/ros1_bridge.git
    ```

1.  Source ROS1 and ROS2 setup bash files. This is one of the only times you'll want to mix setup
    files from different ROS distributions.
    ```
    source ~/catkin_ws_to_bridge/devel/setup.bash
    source ~/colcon_ws/install/setup.bash
    ```

    You should see a warning about the `ROS_DISTRO` variable being previously set to a different
    value.  Now the environment contains references to both distributions which can be verified by
    observing the CMake path.
    ```
    echo $CMAKE_PREFIX_PATH | tr ':' '\n'
    ```

1.  Build the bridge. This may take a while (~30 min) since it is creating mappings between all known message
    and service types.
    ```
    cd ~/ros1_bridge_ws
    colcon build --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=FALSE
    ```

1.  List the mapped message and service pairs.
    ```
    source install/local_setup.bash
    ros2 run ros1_bridge dynamic_bridge --print-pairs
    ```
    This should list the custom mapped services.

    >- `'myworkcell_msgs/srv/ExecuteTrajectory' (ROS 2) <=> 'myworkcell_core/ExecuteTrajectory' (ROS 1)`
    >- `'myworkcell_msgs/srv/LocalizePart' (ROS 2) <=> 'myworkcell_core/LocalizePart' (ROS 1)`
    >- `'myworkcell_msgs/srv/MoveToPose' (ROS 2) <=> 'myworkcell_core/MoveToPose' (ROS 1)`
    >- `'myworkcell_msgs/srv/PlanCartesianPath' (ROS 2) <=> 'myworkcell_core/PlanCartesianPath' (ROS 1)`

    The command should also list the custom mapped topic message. 
    
    >`'myworkcell_msgs/msg/ARMarker' (ROS 2) <=> 'fake_ar_publisher/ARMarker' (ROS 1)`
    
    If you struggle to verify this information in the lengthy terminal output, use `grep` to simplify the terminal output.
    ```
    ros2 run ros1_bridge dynamic_bridge --print-pairs | grep myworkcell
    ```

---
## Run the Demo

### Run the ROS1 nodes

1.  Open a new terminal, source your ROS1 workspace, and run the following launch file.
    ```
    cd ~/catkin_ws_to_bridge
    source devel/setup.bash
    roslaunch myworkcell_support ros2_setup.launch
    ```

    This will launch the motion planning and motion execution nodes is ROS1. Rviz will come up as well.

### Run the ROS1 bridge

1.  In your ros1_bridge_ws ROS2 workspace, source the workspace if you have not already.
    ```
    cd ~/ros1_bridge_ws
    source install/setup.bash
    ```

1.  Export the _ROS_MASTER_URI_ environment variable.
    ```
    export ROS_MASTER_URI=http://localhost:11311
    ```

1.  Run the bridge.
    ```
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
    ```

    We are using `--bridge-all-topics` argument to ensure the `dynamic_bridge` will force a connnection to be made, even
    for topics not used in the ROS2 C++ code. This allows a ROS2 terminal to view all of the ROS1 topics. For example, without the `--bridge-all-topics` argument, a mapped message topic like `ar_pose_marker` would not be bridged since there is not a ROS2 subscriber in in the C++ code.

### Run the ROS2 nodes

1.  Open a new terminal and source your ROS2 workspace.
    ```
    cd ~/colcon_ws
    source install/setup.bash
    ```

1.  Run the python launch file that starts the ROS2 nodes for this application.
    ```
    ros2 launch myworkcell_core workcell.launch.py
    ```
    If the program succeeds you should see the following output:

    > `[INFO] [myworkcell_node-1]: process started with pid [522184]`
    > `[myworkcell_node-1] Got base_frame parameter world`
    > `[myworkcell_node-1] Waiting for client /localize_part`
    > `[myworkcell_node-1] Waiting for client /plan_path`
    > `[myworkcell_node-1] Waiting for client /move_to_pose`
    > `[myworkcell_node-1] Waiting for client /execute_trajectory`
    > `[myworkcell_node-1] Found all services`
    > `[myworkcell_node-1] Requesting pose in base frame: world`
    > `[myworkcell_node-1] Part localized`
    > `[myworkcell_node-1] Planning trajectory`
    > `[myworkcell_node-1] Executing trajectory`
    > `[myworkcell_node-1] Trajectory execution complete`
    > `[INFO] [myworkcell_node-1]: process has finished cleanly [pid 522184]`

    You should also see the robot moving accordingly in the Rviz window.

### Communicate a Publisher/subscriber topic over the bridge:

The information for the ARMarker message is already communicated over the bridge using the ROS1 Service Server `localize_part`. However, we will practice using a custom topic message to confirm the information by subscribing to the `ar_pose_marker` topic from the terminal.

Open a new terminal and source to the ROS2 workspace.
```
source ~/colcon_ws/install/setup.bash
```

List the available ROS2 topics. You should see `ar_pose_marker` among the options.
```
ros2 topic list

```
Print the topic's messages in your terminal.
```
ros2 topic echo /ar_pose_marker
```

Displayed in the terminal, you should see the marker goal pose that was used for motion
planning. The ROS2 workspace called the planner in the previous section called "Run the ROS2 nodes."

-- ---
REMINDER: If purging `controller_manager_msgs` from the `foxy` installation was required to complete Exercise 7.2, reinstall `ros2_control` so the package can be used during other demos and exercises. 
```
sudo apt install ros-foxy-ros2-control
```

<!-- ---
## Issues:
  * **Problem**: The application only runs successfully once, the next run will fail due to a tf
    lookup operation in the vision_node.  This likely happens due to the bridge removing the
    bridging for all `/tf` topics after the application ends.

   * **Solution**: Restart the `ros1_bridge` and then the `workcell.launch.py` application. -->
