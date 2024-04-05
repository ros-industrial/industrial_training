# Motion Planning using C++
>In this exercise, we'll explore MoveIt's C++ interface to programatically move a robot. 


## Motivation
Now that we’ve got a working MoveIt! configuration for your workcell and we’ve played a bit in RViz with the planning tools, let’s perform planning and motion in code. This exercise will introduce you to the basic C++ interface for interacting with the MoveIt! node in your own program. There are lots of ways to use MoveIt!, but for simple applications this is the most straight forward.

## Reference Example
[MoveIt-Cpp tutorial](https://moveit.picknik.ai/humble/doc/tutorials/your_first_project/your_first_project.html)

## 3. Further Information and Resources
 * [MoveIt! Tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html)
 * [MoveIt! home-page](http://moveit.ros.org/)

## Scan-N-Plan Application: Problem Statement
In this exercise, your goal is to modify the `myworkcell_core` node to:

 1. Use the MoveItCpp API to enable planning and execution of the robot from within your C++ program.
 1. Move the robot’s tool frame to the center of the part location as reported by the service call to your vision node.

## Scan-N-Plan Application: Guidance

### MoveItCpp

 For this exercise we will use MoveIt's *MoveItCpp* API, which lets us directly call into the MoveIt libraries from our C++ application. This API is new in ROS2. ROS1 used a similar *MoveGroupInterface* API (still available in ROS2), which uses ROS services and actions to send commands to a standalone MoveIt node.

 1. Add dependencies on the MoveIt packages `moveit_msgs` and `moveit_ros_planning_interface` to `myworkcell_core/CMakeLists.txt`:

    ```
    find_package(moveit_msgs REQUIRED)
    find_package(moveit_ros_planning_interface REQUIRED)

    ...

    ament_target_dependencies(myworkcell_node rclcpp moveit_msgs moveit_ros_planning_interface )
    ```

    and to `package.xml`:

    ```
    <depend>moveit_msgs</depend>
    <depend>moveit_ros_planning_interface</depend>
    ```

 1. Open `myworkcell_core/src/myworkcell_node.cpp`. We'll first add the needed MoveItCpp objects as new class members of the node. Add the following lines in the `private` section of the `ScanNPlan` node:

    ```c++
    moveit_cpp::MoveItCppPtr moveit_cpp_;
    moveit_cpp::PlanningComponentPtr planning_component_;
    moveit_cpp::PlanningComponent::PlanRequestParameters plan_parameters_;
    ```

 1. Add the required includes for these objects at the top of the file:

    ```c++
    #include <moveit/moveit_cpp/moveit_cpp.h>
    #include <moveit/moveit_cpp/planning_component.h>
    ```

 1. Create a new `setup()` function inside `ScanNPlan` after the constructor and above `start`. We'll use this function to initialize the MoveIt objects after the node has started.

    ```c++
    // MoveIt setup
    void setup()
    {
      // Instantiate moveit_cpp
      moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(this->shared_from_this());

      // Planning component associated with a single motion group
      planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>("manipulator", moveit_cpp_);

      // Parameters set on this node
      plan_parameters_.load(this->shared_from_this());
    }
    ```

### Planning

 1. In the `ScanNPlan` class's `start` method, use the response from the `LocalizePart` service to initialize a new `move_target` variable:

    ``` c++
    geometry_msgs::msg::PoseStamped move_target;
    move_target.header.frame_id = base_frame;
    move_target.pose = response->pose;
    ```

    * Make sure to place this code _after_ the call to the vision_node's service.
    * You may need to provide another include for this datatype: `#include <geometry_msgs/msg/pose_stamped.hpp>`.

 1. We'll use the `PlanningComponent` object to plan to this target, but it needs additional information about where the robot will start from and which part of the robot should move to the target. Add the following lines to get the current robot state and the name of end effector link:

    ```c++
    // getting current state of robot from environment
    if (!moveit_cpp_->getPlanningSceneMonitor()->requestPlanningSceneState())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get planning scene");
      return;
    }
    moveit::core::RobotStatePtr start_robot_state = moveit_cpp_->getCurrentState(2.0);

    // Set motion goal of end effector link
    std::string ee_link = moveit_cpp_->getRobotModel()->getJointModelGroup(
        planning_component_->getPlanningGroupName())->getLinkModelNames().back();
    ```

 1. Now we can add the main functionality, the calls to plan and execute:

    ```c++
    planning_component_->setStartState(*start_robot_state);
    planning_component_->setGoal(move_target, ee_link);

    // Now we can plan!
    moveit_cpp::PlanningComponent::PlanSolution plan_solution = planning_component_->plan(plan_parameters_);
    if (!plan_solution)
    {
      RCLCPP_ERROR(this->get_logger(),"Failed to plan");
      return;
    }

    // If planning succeeded, execute the returned trajectory
    bool success = moveit_cpp_->execute("manipulator", plan_solution.trajectory, true);
    if (!success)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to execute trajectory");
      return;
    }
    ```

 1. Currently, MoveIt2 uses many parameters that are not declared ahead of time. To enable this, we have to construct our ROS2 node with an option to automatically declare parameters when they are set in a launch file. Modify the `ScanNPlan` constructor to start with the following:

    ```
    ScanNPlan() : Node("scan_n_plan", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
      if (! this->has_parameter("base_frame"))
      {
        this->declare_parameter("base_frame", "world");
      }
    ```

    Note that the previous `declare_parameter` will now throw an exception if the parameter gets automatically declared which is why we first check if the parameter already exists now.

### Execution

 1. The `plan` and `execute` functions are all that's needed to get your manipulator to move (with the right parameters) but we need to tweak the ROS "spin" behavior to allow those functions to run properly.  Inside the `main` function, insert the following lines before you call `app->start`:

    ```
    // Start spinning in a background thread so MoveIt internals can execute
    std::thread worker{ [app]() { rclcpp::spin(app); } };


    // Perform MoveIt initialization
    app->setup();
    ```

    This lets ROS process callbacks in a separate worker thread while the main thread remains available for us to define our application logic.  This implies we can no longer call any spin functions in the main thread now. Replace the call to `spin_until_future_complete` in the `start` function with a function to simply wait for the future to be ready:

    ```diff
    -if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS)
    +if (future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
    ```

    Finally, before calling rclcpp::shtudown(), wait for the thread to rejoin the main process.

    ```
    worker.join();
    ```

### Launch files and testing

 1. Open your `workcell.launch.py` and replace it with the following contents:

    ```
    import os
    import yaml
    import xacro
    from launch import LaunchDescription
    from launch_ros.actions import Node
    from ament_index_python import get_package_share_directory

    def get_package_file(package, file_path):
        """Get the location of a file installed in an ament package"""
        package_path = get_package_share_directory(package)
        absolute_file_path = os.path.join(package_path, file_path)
        return absolute_file_path

    def load_file(file_path):
        """Load the contents of a file into a string"""
        try:
            with open(file_path, 'r') as file:
                return file.read()
        except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
            return None

    def load_yaml(file_path):
        """Load a yaml file into a dictionary"""
        try:
            with open(file_path, 'r') as file:
                return yaml.safe_load(file)
        except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
            return None

    def generate_launch_description():
        xacro_file = get_package_file('myworkcell_support', 'urdf/workcell.urdf.xacro')
        srdf_file = get_package_file('myworkcell_moveit_config', 'config/myworkcell.srdf')
        kinematics_file = get_package_file('myworkcell_moveit_config', 'config/kinematics.yaml')
        ompl_config_file = get_package_file('myworkcell_moveit_config', 'config/ompl_planning.yaml')
        joint_limits_file = get_package_file('myworkcell_moveit_config','config/joint_limits.yaml')
        moveit_controllers_file = get_package_file('myworkcell_moveit_config', 'config/moveit_controllers.yaml')

        robot_description = xacro.process_file(xacro_file).toprettyxml(indent='  ')
        robot_description_semantic = load_file(srdf_file)
        kinematics_config = load_yaml(kinematics_file)
        ompl_config = load_yaml(ompl_config_file)
        joint_limits_config = load_yaml(joint_limits_file)

        # Setting up MoveitCpp configuration parameters
        moveit_controllers = load_yaml(moveit_controllers_file)
        trajectory_execution = {
            'moveit_manage_controllers': True,
            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
            'trajectory_execution.allowed_goal_duration_margin': 0.5,
            'trajectory_execution.allowed_start_tolerance': 0.01
        }
        planning_scene_monitor_config = {
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True
        }

        moveit_cpp_config = yaml.load("""
            planning_scene_monitor_options:
              name: "planning_scene_monitor"
              robot_description: "robot_description"
              joint_state_topic: "/joint_states"
              attached_collision_object_topic: "/moveit_cpp/planning_scene_monitor"
              publish_planning_scene_topic: "/moveit_cpp/publish_planning_scene"
              monitored_planning_scene_topic: "/moveit_cpp/monitored_planning_scene"
              wait_for_initial_state_timeout: 10.0

            planning_pipelines:
              #namespace: "moveit_cpp"  # optional, default is ~
              pipeline_names: ["ompl"]

            plan_request_params:
              planning_time: 10.0
              planning_attempts: 3
              planning_pipeline: ompl
              max_velocity_scaling_factor: 0.5
              max_acceleration_scaling_factor: 0.5

            # octomap parameters (when used)
            octomap_frame: world
            octomap_resolution: 0.01
            max_range: 5.0""")

        return LaunchDescription([
            Node(
                name='myworkcell_node',
                package='myworkcell_core',
                executable='myworkcell_node',
                output='screen',
                parameters=[
                    {
                        'base_frame': 'world',
                        'robot_description': robot_description,
                        'robot_description_semantic': robot_description_semantic,
                        'robot_description_kinematics': kinematics_config,
                        'robot_description_planning' : joint_limits_config,
                        'planning_pipelines': ['ompl'],
                        'ompl': ompl_config
                    },
                    moveit_cpp_config,
                    moveit_controllers,
                    trajectory_execution,
                    planning_scene_monitor_config,
                ],
            ),
            Node(
                name='fake_ar_publisher_node',
                package='fake_ar_publisher',
                executable='fake_ar_publisher_node',
                output='screen',
            ),
            Node(
                name='vision_node',
                package='myworkcell_core',
                executable='vision_node',
                output='screen',
            ),
        ])
    ```

    * Note that this uses the same set of helper functions as previous launch files
    * All of the extra complexity is used to build the set of parameters for the `myworkcell_node` node.

 1. Now let's test the system!

    ``` bash
    colcon build
 
    ros2 launch myworkcell_moveit_config demo.launch.py
    ros2 launch myworkcell_support workcell.launch.py
    ```
