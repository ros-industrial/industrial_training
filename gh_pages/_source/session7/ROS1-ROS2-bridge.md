# ROS1-ROS2 Bridge Demo

## Introduction

This is a system integration exercise to demonstrate operation of the ROS1-ROS2 topic and service
bridge. Using the bridge does not require anything different when developing either ROS1 or ROS2
software, so much of the required code is written in a template. 

This demo is an example of a system where a ROS2 application needs to call a planner that only
exists as a ROS1 package. Specifically, this demo is calling the Descartes motion planner,
as seen in exercise 4.1. On the ROS1 side, services are provided to generate motion plans and to
execute them. In order to use these custom services, the bridge needs to be compiled from source 
with the message definitions in the build environment, which is the bulk of the complexity in this 
exercise.

## Building the Demo

This exercise uses the ROS1 bridge to call ROS nodes from ROS2 nodes and therefore the build
procedure is somewhat involved.

### Create a ROS workspace for exercise 4.1

1.  Create a catkin workspace for the exercise 4.1 ROS packages and dependencies
    ```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    ```

1.  Create a symlink to exercise 4.1 in the training repo
    ```
    cd ~/catkin_ws/src
    ln -s ~/industrial_training/exercises/4.1/ros1/src demo
    ```

1.  Clone additional dependencies
    ```
    git clone --branch melodic-devel https://github.com/ros-industrial-consortium/descartes.git
    git clone --branch kinetic-devel https://github.com/ros-industrial/universal_robot.git
    git clone --branch master https://github.com/ros-industrial/fake_ar_publisher.git
    ```

1.  Source and build exercise
    ```
    . /opt/ros/noetic/setup.bash
    catkin build    
    ```

### Create the ROS2 workspace

1.  Create a colcon workspace for the exercise 7.2 ROS packages and dependencies. We will copy a
template with part of the exercise created for you.
    ```
    mkdir -p ~/colcon_ws/src
    cd ~/colcon_ws/src
    ```
    ```
    cp ~/industrial_training/exercises/7.2/template_ws/src ~/colcon_ws/src
    ```

1.  Add the message mapping parameters to the `message_mappings.yaml`.
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

1.  Add the definitions and declarations for the services that will be communicated over the bridge
    ```C++
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
    ```

1.  Add a `start()` method that will call the service servers from ROS. The final class from this step
    and the previous should look like below. 
    ```C++
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
      //RCLCPP_INFO(this->get_logger(),"Requesting pose in base frame: %s", base_frame.c_str()); // TODO: enable this when   printing to console works again
  
      std::shared_future<LocalizePart::Response::SharedPtr> result_future = vision_client_->async_send_request(req);
      if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),result_future) !=   rclcpp::executor::FutureReturnCode::SUCCESS)
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
      if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),move_future) !=   rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),"Failed to move to target");
        return false;
      }
  
      // planning cartesian path
      PlanCartesianPath_Request::SharedPtr plan_req = std::make_shared<PlanCartesianPath::Request>();
      plan_req->pose = move_target;
      std::cout<<"Planning trajectory"<<std::endl;
      std::shared_future<PlanCartesianPath_Response::SharedPtr> plan_future = cartesian_client_->async_send_request(plan_req);
      if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),plan_future) !=   rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),"Could not plan for path");
        return false;
      }
  
      // executing
      ExecuteTrajectory::Request::SharedPtr exec_req = std::make_shared<ExecuteTrajectory::Request>();
      exec_req->trajectory = plan_future.get()->trajectory;
      std::cout<<"Executing trajectory"<<std::endl;
      std::shared_future<ExecuteTrajectory::Response::SharedPtr> exec_future = exec_traj_client_->async_send_request(exec_req);
      if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), exec_future) !=   rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(this->get_logger(),"Failed to execute trajectory");
        return false;
      }
      std::cout<<"Trajectory execution complete"<<std::endl;
      return true;
    }

    //***FILL CODE HERE TO DECLARE SERVICES
    rclcpp::Client<myworkcell_msgs::srv::LocalizePart>::SharedPtr vision_client_;
    rclcpp::Client<myworkcell_msgs::srv::PlanCartesianPath>::SharedPtr cartesian_client_;
    rclcpp::Client<myworkcell_msgs::srv::MoveToPose>::SharedPtr move_client_;
    rclcpp::Client<myworkcell_msgs::srv::ExecuteTrajectory>::SharedPtr exec_traj_client_;
    ```

1.  Add a main method that starts ROS2 and calls the `start()` method after instantiation
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

### Build the ROS1 Bridge 

1.  Create the ros1_bridge workspace. We build the bridge in a separate workspace because it needs
    to see both ROS1 and ROS2 packages in its environment and we want to make sure our application
    workspaces only see the packages from the distribution they are in.
    ```
    mkdir -p ~/ros1_bridge_ws/src
    cd ~/ros1_bridge_ws/src
    ```

1.  Clone the ROS1 bridge into your ROS2 workspace, selecting the branch that matches your ROS
    release
    ```
    git clone -b foxy https://github.com/ros2/ros1_bridge.git
    ```

1.  Source ROS1 and ROS2 setup bash files. This is one of the only times you'll want to mix setup
    files from different ROS distributions.
    ```
    . ~/catkin_ws/devel/setup.bash
    . ~/colcon_ws/install/setup.bash
    ```

    You should see a warning about the `ROS_DISTRO` variable being previously set to a different
    value.  Now the environment contains references to both distributions which can be verified by
    observing the CMake path:
    ```
    echo $CMAKE_PREFIX_PATH | tr ':' '\n'
    ```

1.  Build the bridge. This may take a while since it is creating mappings between all known message
    and service types.
    ```
    colcon build --packages-select ros1_bridge --cmake-force-configure --cmake-args -DBUILD_TESTING=FALSE
    ```

1.  List the mapped message and service pairs
    ```
    source install/local_setup.bash
    ros2 run ros1_bridge dynamic_bridge --print-pairs
    ```
    This should list the custom `myworkcell_msgs(ROS2) <-> myworkcell_core` mapped services

---
## Run the Demo

### Run the ROS nodes

1.  In another sourced terminal, run the following launch file.
    ```
    roslaunch myworkcell_support ros2_setup.launch
    ```

    This will launch the motion planning and motion execution nodes is ROS.  Rviz will come up as well.

### Run the ROS1 bridge

1.  In your ros1_bridge_ws ROS2 workspace, source the workspace if you haven't
    ```
    cd ~/ros1_bridge_ws
    source install/setup.bash
    ```

1.  Export the _ROS_MASTER_URI_ environment variable
    ```
    export ROS_MASTER_URI=http://localhost:11311
    ```

1.  Run the bridge
    ```
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
    ```

    An alternative to the `dynamic_bridge` would be to use the `parameter_bridge` to define the topic matchings between ROS and ROS2. We are using `--bridge_all_topics` argument to ensure the `dynamic_bridge` will force the connnection to be made so a ROS2 terminal can view all of the ROS topics. A mapped message topic (like `ar_marker pose`) would not be bridged otherwise since there is not a ROS2 subscriber in in the C++ code.

### Run the ROS2 nodes

1.  Open a new terminal and source your ROS2 workspace
    ```
    cd ~/colcon_ws
    source install/setup.bash
    ```

1.  Run the python launch file that starts the ROS2 nodes for this application
    ```
    ros2 launch myworkcell_core workcell.launch.py
    ```
    If the program succeeds you should see the following output:

    > [myworkcell_node-2] Got base_frame parameter world  
    > [myworkcell_node-2] Waiting for client /localize_part  
    > [myworkcell_node-2] Waiting for client /plan_path  
    > [myworkcell_node-2] Waiting for client /move_to_pose  
    > [myworkcell_node-2] Waiting for client /execute_trajectory  
    > [myworkcell_node-2] Found all services  
    > [myworkcell_node-2] Requesting pose in base frame: world  
    > [myworkcell_node-2] Part localized  
    > [myworkcell_node-2] Planning trajectory  
    > [myworkcell_node-2] Executing trajectory  
    > [myworkcell_node-2] Trajectory execution complete  

    You should also see the robot moving accordingly in the Rviz window.

### Communicate a Publisher/subscriber topic over the bridge:

The information for the AR Marker is already communicated over the bridge using the ROS1 Service 
Server `localize_part`. However, we will practice using a custom topic message to confirm the
information by subscribing to the `ar_marker_pose` topic from the terminal.

Open a new terminal and source to the ROS2 workspace
```
. ~/colcon_ws/install/setup.bash
```

List the available ROS2 topics. You should see `ar_marker_pose` among the options.
```
ros2 topic list

```
Echo to the topic in your terminal
```
ros2 topic echo /ar_marker_pose
```

You should see the marker goal pose, that was used for motion planning in the previous section, displayed in the terminal.

---
## Issues:
  * **Problem**: The application only runs successfully once, the next run will fail due to a tf
    lookup operation in the vision_node.  This likely happens due to the bridge removing the
    bridging for all `/tf` topics after the application ends.

   * **Solution**: Restart the `ros1_bridge` and then the `workcell.launch.py` application.
