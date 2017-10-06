# Application Structure
>In this exercise, we'll take a look at all the packages and files that will be used during the completion of these exercises.

## Acquire the Workspace

```
cd ~/industrial_training/exercises/Descartes_Planning_and_Execution
cp -r template_ws ~/descartes_ws
cd ~/descartes_ws/src
catkin_init_workspace
cd ..
rosinstall src
catkin build
source devel/setup.bash
```

## List All the Packages in the Application
```
cd ~/descartes_ws/src
ls -la
```

 * `plan_and_run` :           Contains the source code for the `plan_and_run` application.  You'll be completing the exercises by editing source files in this package
 * `ur5_demo_moveit_config` : Contains support files for planning and execution robot motions with Moveit.  This package was generated with the Moveit Setup Assistant
 * `ur5_demo_support` :       Provides the robot definition as a URDF file.  This URDF is loaded at run time by our `plan_and_run` application.
 * `ur5_demo_descartes` :     Provides a custom Descartes Robot Model for the UR5 arm.  It uses a Inverse-Kinematics closed form solution; which is significantly faster than the numerical approach used by the **MoveitStateAdapter**.

## The `plan_and_run` package

```
roscd plan_and_run
ls -la
```

 * `src` :     Application source files. 
  * `src/demo_application.cpp` : A class source file that contains the application implementation code.
  * `src/plan_and_run.cpp`     : The application main access point. It invokes all the tasks in the application and wraps them inside the "`main`" routine.
  * `src/tasks`                : A directory that contains all of the source files that you'll be editing or completing as you make progress through the exercises.
 * `include` : Header files
  * `include/plan_and_run/demo_application.h` : Defines the application skeleton and provides a number of global variables for passing data at various points in the exercises.
 * `launch`: Launch files needed to run the application
  * `launch/demo_setup.launch` : Loads `roscore`, `moveit` and the runtime resources needed by our application.  
  * `launch/demo_run.launch` : Starts our application main executable as a ROS node.
 * `config`: Directory that contains non-critical configuration files.

## Main Application Source File

 In the "`plan_and_run/src/plan_and_run_node.cpp`" you'll find the following code:

``` c++
int main(int argc,char** argv)
{
  ros::init(argc,argv,"plan_and_run");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // creating application
  plan_and_run::DemoApplication application;

  // loading parameters
  application.loadParameters();

  // initializing ros components
  application.initRos();

  // initializing descartes
  application.initDescartes();

  // moving to home position
  application.moveHome();

  // generating trajectory
  plan_and_run::DescartesTrajectory traj;
  application.generateTrajectory(traj);


  // planning robot path
  plan_and_run::DescartesTrajectory output_path;
  application.planPath(traj,output_path);

  // running robot path
  application.runPath(output_path);

  // exiting ros node
  spinner.stop();

  return 0;
}
```

  In short, this program will run through each exercise by calling the corresponding function from the `application` object.  For instance, in order to initialize Descartes the program calls `application.iniDescartes()`.  Thus each exercise consists of editing the source file where that exercise is implemented, so for `application.initDescartes()` you'll be editing the `plan_and_run/src/tasks/init_descartes.src` source file.


## The DemoApplication Class

  In the header file "`plan_and_run/include/plan_and_run/demo_application.h`" you'll find the definition for the application's main class along with several support constructs. Some of the important components to take notice of are as follows:

  * Program Variables: Contain hard coded values that are used at various points in the application.

``` c++
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
const std::string EXECUTE_TRAJECTORY_SERVICE = "execute_kinematic_path";
const std::string VISUALIZE_TRAJECTORY_TOPIC = "visualize_trajectory_curve";
const double SERVICE_TIMEOUT = 5.0f; // seconds
const double ORIENTATION_INCREMENT = 0.5f;
const double EPSILON = 0.0001f;
const double AXIS_LINE_LENGHT = 0.01;
const double AXIS_LINE_WIDTH = 0.001;
const std::string PLANNER_ID = "RRTConnectkConfigDefault";
const std::string HOME_POSITION_NAME = "home";
```

 * Trajectory Type: Convenience type that represents an array of Descartes Trajectory Points.

``` c++
typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTrajectory;
```


 * **DemoConfiguration Data Structure**:  Provides variables whose values are initialize at run-time from corresponding ros parameters.

``` c++
struct DemoConfiguration
{
  std::string group_name;                 /* Name of the manipulation group containing the relevant links in the robot */
  std::string tip_link;                   /* Usually the last link in the kinematic chain of the robot */
  std::string base_link;                  /* The name of the base link of the robot */
  std::string world_frame;                /* The name of the world link in the URDF file */
  std::vector<std::string> joint_names;   /* A list with the names of the mobile joints in the robot */


  /* Trajectory Generation Members:
   *  Used to control the attributes (points, shape, size, etc) of the robot trajectory.
   *  */
  double time_delay;              /* Time step between consecutive points in the robot path */
  double foci_distance;           /* Controls the size of the curve */
  double radius;                  /* Controls the radius of the sphere on which the curve is projected */
  int num_points;                 /* Number of points per curve */
  int num_lemniscates;            /* Number of curves*/
  std::vector<double> center;     /* Location of the center of all the lemniscate curves */
  std::vector<double> seed_pose;  /* Joint values close to the desired start of the robot path */

  /*
   * Visualization Members
   * Used to control the attributes of the visualization artifacts
   */
  double min_point_distance;      /* Minimum distance between consecutive trajectory points. */
};
```

 * **DemoApplication Class**:  Main component of the application which provides functions for each step in our program.  It also contains several constructs that turn this application into a ROS node.

``` c++
class DemoApplication
{
public:
  /*  Constructor
   *    Creates an instance of the application class
   */
  DemoApplication();
  virtual ~DemoApplication();

  /* Main Application Functions
   *  Functions that allow carrying out the various steps needed to run a
   *  plan an run application.  All these functions will be invoked from within
   *  the main routine.
   */

  void loadParameters();
  void initRos();
  void initDescartes();
  void moveHome();
  void generateTrajectory(DescartesTrajectory& traj);
  void planPath(DescartesTrajectory& input_traj,DescartesTrajectory& output_path);
  void runPath(const DescartesTrajectory& path);

protected:

  /* Support methods
   *  Called from within the main application functions in order to perform convenient tasks.
   */

  static bool createLemniscateCurve(double foci_distance, double sphere_radius,
                                    int num_points, int num_lemniscates,
                                    const Eigen::Vector3d& sphere_center,
                                    EigenSTL::vector_Affine3d& poses);

  void fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj,
                                              trajectory_msgs::JointTrajectory& out_traj);

  void publishPosesMarkers(const EigenSTL::vector_Affine3d& poses);


protected:

  /* Application Data
   *  Holds the data used by the various functions in the application.
   */
  DemoConfiguration config_;



  /* Application ROS Constructs
   *  Components needed to successfully run a ros-node and perform other important
   *  ros-related tasks
   */
  ros::NodeHandle nh_;                        /* Object used for creating and managing ros application resources*/
  ros::Publisher marker_publisher_;           /* Publishes visualization message to Rviz */
  ros::ServiceClient moveit_run_path_client_; /* Sends a robot trajectory to moveit for execution */



  /* Application Descartes Constructs
   *  Components accessing the path planning capabilities in the Descartes library
   */
  descartes_core::RobotModelPtr robot_model_ptr_; /* Performs tasks specific to the Robot
                                                     such IK, FK and collision detection*/
  descartes_planner::SparsePlanner planner_;      /* Plans a smooth robot path given a trajectory of points */

};
```

## Application Launch File
 This file starts our application as a ROS node and loads up the necessary parameters into the ROS parameter server. Observe how this is done by opening the "`plan_and_run/launch/demo_run.launch`" file:

``` xml
<launch>
  <node name="plan_and_run_node" type="plan_and_run_node" pkg="plan_and_run" output="screen">
    <param name="group_name" value="manipulator"/>
    <param name="tip_link" value="tool"/>
    <param name="base_link" value="base_link"/>
    <param name="world_frame" value="world"/>
    <param name="trajectory/time_delay" value="0.1"/>
    <param name="trajectory/foci_distance" value="0.07"/>
    <param name="trajectory/radius" value="0.08"/>
    <param name="trajectory/num_points" value="200"/>
    <param name="trajectory/num_lemniscates" value="4"/>
    <rosparam param="trajectory/center">[0.36, 0.2, 0.1]</rosparam>
    <rosparam param="trajectory/seed_pose">[0.0, -1.03, 1.57 , -0.21, 0.0, 0.0]</rosparam>
    <param name="visualization/min_point_distance" value="0.02"/>
  </node>
</launch>
```

 * Some of the important parameters are explained as follows:
  * group_name: A namespace that points to the list of links in the robot that are included in the arm's kinematic chain (base to end-of-tooling links).  This list is defined in the `ur5_demo_moveit_config` package.
  * tip_link: Name of the last link in the kinematic chain, usually the tool link.
  * base_link: Name for the base link of the robot.
  * world_frame: The absolute coordinate frame of reference for all the objects defined in the planning environment.
 * The parameters under the "`trajectory`" namespace are used to generate the trajectory that is feed into the Descartes planner.
  * trajectory/seed_pose: This is of particular importance because it is used to indicate preferred start and end joint configurations of the robot when planning the path.  If a ''seed_pose'' wasn't specified then planning would take longer since multiple start and end joint configurations would have to be taken into account, leading to multiple path solutions that result from combining several start and end poses. 
 
