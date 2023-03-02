Motion Planning with Tesseract
==============================
In this exercise we will fill in the appropriate pieces of code to create a motion planning pipeline using Tesseract. The end goal of this will be to implement a "scan and plan" application. Much of the code from the exercise is from our `Scan N Plan Workshop <https://github.com/ros-industrial-consortium/scan_n_plan_workshop>`_.

Prepare New Workspace:
----------------------
We will create a new workspace, since this exercise does not overlap with the previous exercises.

#. Disable automatic sourcing of your previous workspace in your ``.bashrc`` if you had any.

    .. Note:: This means you'll need to manually source the setup file from your new colcon workspace in each new terminal window.

#. Source ROS 2 into your environment

    .. code-block:: bash

		source /opt/ros/foxy/setup.bash

    .. Note:: If you are using a VM provided by us, please skip to step 7 as a directory called ``tesseract_ws`` should already exist with the dependencies already installed for you.

#. Copy the template workspace layout and files:

	.. code-block:: bash

		mkdir -p ~/tesseract_ws/src
		cp -r ~/industrial_training/exercises/8.0/template/. ~/tesseract_ws/src
		cd ~/tesseract_ws/

#. Install ``taskflow`` from the ROS-I PPA

	.. code-block:: bash

		sudo add-apt-repository ppa:ros-industrial/ppa
		sudo apt-get update
		sudo apt-get install taskflow

#. Install the ROS 2 dependencies (this may take a while)

	.. code-block:: bash

		cd ~/tesseract_ws
		vcs import src < src/dependencies_tesseract.repos
		vcs import src < src/dependencies.repos
		vcs import src < src/snp_automate_2022/dependencies.repos
		rosdep install --from-paths src --ignore-src -r -y

#. Initialize and Build this new workspace (this may take a little while)

    	.. code-block:: bash

		cd ~/tesseract_ws
		colcon build --cmake-args -DTESSERACT_BUILD_FCL=OFF

#. Source the workspace

    	.. code-block:: bash

    		source ~/tesseract_ws/install/setup.bash

#. Import the new workspace into your QTCreator IDE (if using QTCreator):

   * In QTCreator: `File -> New File or Project -> Other Project -> ROS Workspace -> ~/tesseract_ws`
   * Don't forget to check the ROS distro and the build system!

Intro (Review Existing Code)
----------------------------
Most of the infrastructure for a ROS node has already been completed for you; the focus of this exercise are Tesseract planner profiles and taskflow generators. You will notice that many files and packages are already provided for you. You could run the application as is, but you may get errors. At this time we will explore the source code that has been provided. The following are highlights of what is included.

#. ``snp_automate_2022/config/worcell_plugins.yaml``:
	* This file contains all of the kinematic plugins and contact manager plugins for our application. A kinematic plugin configuration file like this is required to use Tesseract. Take a look at ``workcell.srdf`` to see how it gets incorporated into the project.

#. ``snp_motion_planning/src/planner_profiles.hpp``:
	* This file contains the planner profiles used to create our motion plan. Currently, only the Simple Planner profile is fully populated. This is one of the main files we will be editing in our exercise.

#. ``snp_motion_planning/src/planning_server.cpp``:
	* This is where our custom planner profiles will be used by our application. Take a look at the ``createProgram()`` method. This method takes in the toolpath rasters and constructs motion plan requests in a manner usable by Tesseract. These motions include freespace motions, transition motions, and raster (process) motions. The order that they are added is the same order that they will be returned in.

#. ``snp_motion_planning/src/taskflow_generators.hpp``:
	* This file creates taskflow graphs for planning transition, raster, and freespace motions using our planners. This is another main file we will be editing.

Fill in the Code
----------------
The planner profiles tell Tesseract how to use a given motion planner with specific configurations. These configurations can range from timeout limits to specific pose sampling. For this exercise, we will be implementing the profiles for Descartes, OMPL, and TrajOpt. 

Currently, only the Simple Planner is set up. Try running the application and see how the motion plan performs.

	.. code-block:: bash

		ros2 launch snp_automate_2022 start.launch.xml

You should see an Rviz window appear with a robot on a table. Click `Get Detailed Scan` to see a model of our work surface appear on the table. Use the `Polygon Selection Tool` at the top to select a region on the work surface. 

Find the `snp_tpp_app` window that should also have appeared when you launched the application. We can use this to select different tool path planners and modifiers. Add ``ROISelectionMeshModifier`` and ``PlaneSlicerRasterPlanner``. You should see more options appear on the screen after. Feel free to play around with these and see how they affect your tool path plan. For the `Tool Path Modifier` we recommend adding ``SnakeOrganizationModifier`` and ``MovingAverageOrientationSmoothingModifier``.

After making changes on the `snp_tpp_app` return to Rviz and click `Generate Tool Path Plan`. You should now see waypoints appear in your selected region. When you are satisfied with the waypoints, click `Generate Motion Plan` (this may take a few minutes). Before beginning the exercise, you'll notice that the motion plans will all fail.

There should also be a `joint_state_publisher_gui` on your screen. Feel free to play around with it as well to create different start states. Note that the motion plan will fail if your start state is in collision.

When a motion plan succeeds you will see a message saying ``perform motion planning completed!``. To see the motion plan play, navigate to the ``TesseractWorkbench`` panel at the bottom right of the Rviz screen and go to the ``Trajectory`` tab. Expand ``general`` and click on ``Trajectory Set``. A play button should appear at the bottom of the panel. You can click it to play through your motion plan. You can also expand the trajectory to see the various states the robot moves to during its motion. 

.. Note:: If the application fails to create a motion plan, try playing around with the settings in `snp_tpp_app`. You may need to change the line and point spacing. 

Implement the Descartes Planner Profile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. Create the planner profile:
	
   Within ``snp_motion_planning/src/planner_profiles.hpp``, find the section

   .. code-block:: c++

      /* =======================
       * Fill Code: DESCARTES 
       * =======================*/

   We will be replacing the current contents of the method. We must fist set up some configurations we want our Descartes planner to follow. The following block specifies the number of threads we need for the planner, if we allow redundant joint solutions, and whether or not to allow collisions.
   Replace the contents with the following:

   .. code-block:: c++

   	auto profile = std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<FloatType>>();
	profile->num_threads = static_cast<int>(std::thread::hardware_concurrency());
	profile->use_redundant_joint_solutions = false;
	profile->allow_collision = false;
	profile->enable_collision = true;
	profile->enable_edge_collision = false;

   Now we will also specify our state and edge evaluators. The state evaluator looks at a given state and gives the state both a cost and a pass or fail. A few things we may choose to evaluate are whether or not the state is valid, the cost of the state, and any biases we may want to give it. The edge evaluator works similarly but evaluates a transition between states. 

   Copy and past the following below the previous block:

   .. code-block:: c++

	// Use the default state and edge evaluators
	profile->state_evaluator = nullptr;
	profile->edge_evaluator = [](const tesseract_planning::DescartesProblem<FloatType>& prob) ->
	typename descartes_light::EdgeEvaluator<FloatType>::Ptr {
		auto eval = std::make_shared<descartes_light::CompoundEdgeEvaluator<FloatType>>();

		// Nominal Euclidean distance
		eval->evaluators.push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>());

		return eval;
	};
      	profile->vertex_evaluator = nullptr;

   Finally, we set the ``target_pose_sampler`` which takes a given function for sampling. In our example, we specify our pose sampling to allow any rotation along the z-axis as it will not impact our final results. Note that Descartes can only work in discrete space so we are only sampling at increments of 10 degrees around the z-axis.

   Copy and past the following below the previous block:

   .. code-block:: c++

	profile->target_pose_sampler =
	std::bind(tesseract_planning::sampleToolZAxis, std::placeholders::_1, 10.0 * M_PI / 180.0);

	return profile;

#. Add the planner to the planning server:
   
   Within ``snp_motion_planning/src/planning_server.cpp``, find the section

   .. code-block:: c++

      /* ========================================
       * Fill Code: ADD CUSTOM PLANNER PROFILES
       * ========================================*/

   Copy and past the following below:

   .. code-block:: c++

   	profile_dict_->addProfile<tesseract_planning::DescartesPlanProfile<float>>(
          tesseract_planning::profile_ns::DESCARTES_DEFAULT_NAMESPACE, PROFILE, createDescartesPlanProfile<float>());

   This line adds your new custom planning profile to the planning server for our motion plan.

#. Add the planner to the taskflow:
   
   Navigate to ``snp_motion_planning/taskflow_generators.hpp`` and find the method ``ctor()`` within the class ``CartesianMotionPipelineTask``. Find the following block inside

   .. code-block:: c++

      /* ========================================
       * Fill Code: CREATE CUSTOM PLANNER NODES
       * ========================================*/

   Below the block add the following

   .. code-block:: c++

      boost::uuids::uuid descartes_planner_task =
            addNode(std::make_unique<tesseract_planning::DescartesMotionPlannerTask>(output_keys_[0], output_keys_[0],
            false));

   Now we have created nodes for our planner. Find 

   .. code-block:: c++

      /* =======================
       * Fill Code: EDIT EDGES
       * =======================*/

   in the same method and fill in the code to add edges between our nodes. Will need to replace 

   .. code-block:: c++

      addEdges(min_length_task, { contact_check_task });

   with

   .. code-block:: c++

      addEdges(min_length_task, { descartes_planner_task });

   and then add the line

   .. code-block:: c++

      addEdges(descartes_planner_task, { error_task, contact_check_task });

   We have now added Descartes to our raster taskflow. 

   .. Note:: Pay attention to how the graph's edges and vertices are connected. We have already included post-collision checking for the simple planner and time parameterization. Play around with removing one or both of those and observe how your motion plan changes. 

#. Run the application:

   Now let's try running our application. Build and source your workspace then run the following

   .. code-block:: bash

      ros2 launch snp_automate_2022 start.launch.xml

   How does the motion plan look? Does it fail to plan often? Does the motion look smooth? 

   Notice that this implementation in the taskflow uses Descartes to resample all waypoints and solves for that single raster again after a global Descartes has already been run. We will fix this later.

   .. Note:: The default starting joint state is likely to be in collision. This will cause all motion plans to fail. Try changing ``joint_3_u`` to about 1.57 to avoid this. If your selected region is too small and/or the waypoints are too far apart from each other, it may also fail to create a motion plan. We recommend lowering the point spacing to 0.010. 

   .. Note:: When re-building your workspace, you may find it useful to only build the package you've edited instead of the entire workspace. You can do this by using the ``--packages-select`` flag with ``colcon``. 

Implement the TrajOpt Planner Profiles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. Create the planner profiles:
 
   In ``planner_profiles.hpp`` find the section

   .. code-block:: c++

      /* ==========================
       * Fill Code: TRAJOPT PLAN
       * ==========================*/

   TrajOpt is a planner that creates a nonlinear optimization problem to solve until it converges on a solution. As this planner does not have any knowledge of time, it only looks at adjacent states while planning. There are three different profiles you can adjust to setup a TrajOpt planner: plan, composite, and solver. In this application we will be customizing the plan and composite profiles.

   We will begin by filling out the plan profile which focuses on how individual waypoints are handled. Below the above block replace the method's contents with the following code

   .. code-block:: c++

	auto profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
	profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 5.0);
	profile->cartesian_coeff(5) = 0.0;
	return profile;

   This method adds a vector of cost constraints on the cartesian axes of the waypoints in order to make certain motions more or less expensive than others. Here, we have costs in all directions except around the z-axis as rotation in the z-axis will not affect our outcomes. 

   Locate the section

   .. code-block:: c++

     /* ==============================
      * Fill Code: TRAJOPT COMPOSITE
      * ==============================*/

   Now we will create the TrajOpt composite profile. Replace that method's code with the following

   .. code-block:: c++

	auto profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
	profile->smooth_velocities = false;

	profile->acceleration_coeff = Eigen::VectorXd::Constant(6, 1, 10.0);
	profile->jerk_coeff = Eigen::VectorXd::Constant(6, 1, 20.0);

	profile->collision_cost_config.enabled = true;
	profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
	profile->collision_cost_config.safety_margin = 0.010;
	profile->collision_cost_config.safety_margin_buffer = 0.010;
	profile->collision_cost_config.coeff = 10.0;

	profile->collision_constraint_config.enabled = false;

	return profile;

   Notice that the composite profile takes more parameters into account than the plan profile. Unlike the plan profile, which only looks at one waypoint at a time, the composite profile looks at the whole motion. You can add costs on velocity, acceleration, and jerk as well as specify how collision checking is to be handled.

#. Add the planners to the planning server:
   
   Go back to ``planning_server.cpp`` and add our two new custom profiles to the server

   .. code-block:: c++

      profile_dict_->addProfile<tesseract_planning::TrajOptPlanProfile>(
          tesseract_planning::profile_ns::TRAJOPT_DEFAULT_NAMESPACE, PROFILE, createTrajOptToolZFreePlanProfile());
      profile_dict_->addProfile<tesseract_planning::TrajOptCompositeProfile>(
          tesseract_planning::profile_ns::TRAJOPT_DEFAULT_NAMESPACE, PROFILE, createTrajOptProfile());

#. Add the planners to the taskflow:

   Return to ``taskflow_generators.hpp``. As Trajopt will be used for transition, freespace, and process planning taskflows, we will need to modify the ``ctor()`` method in ``FreespaceMotionPipelineTask``, ``TransitionMotionPipelineTask``, and ``CartesianMotionPipelineTask``.

   Within ``FreespaceMotionPipelineTask`` add the following to create a new node

   .. code-block:: c++

      // Setup TrajOpt
      boost::uuids::uuid trajopt_planner_task = addNode(
         std::make_unique<tesseract_planning::TrajOptMotionPlannerTask>(output_keys_[0], output_keys_[0], false));

   Now we need to connect our node through edges. Find where the edges are created and add the following line

   .. code-block:: c++

      addEdges(trajopt_planner_task, { error_task, contact_check_task });

   You will also need to modify the edge connecting ``min_length_task`` to ``contact_check_task`` and instead have it connect to our new ``trajopt_planner_task``.
   
   Now navigate down to ``TransitionMotionPipelineTask``. You will need to add the same line as before to create the TrajOpt node. For the edges, change ``min_length_task`` to again connect to ``trajopt_planner_task`` and then add the following line to conenct ``trajopt_planner_task`` to ``error_task`` and ``contact_check_task``.

   ..code-block:: c++

     addEdges(trajopt_planner_task, { error_task, contact_check_task });

   Scroll down to ``CartesianMotionPipelineTask`` and make the same changes to add the TrajOpt node and edges. For the edges, we again want ``min_length_task`` connected to ``trajopt_planner_task`` and ``trajopt_planner_task`` connected to both ``error_task`` and ``contact_check_task``. Additionally, you shoud edit the edge from ``descartes_planner_task`` to go to ``trajopt_planner_task`` instead of ``contact_check_task``.

   Now our TrajOpt planners are connected to our taskflow!

#. Run the application:

   Now try running the application again and notice how our robot's motion plan has changed. Don't forget to build and source your workspace!

   Note that it still might fail frequently since there is nothing to generate good freespace or tranistion motion seeds.

Implement the OMPL Planner Profile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. Create the planner profile:
   
   Go back to ``planner_profiles.hpp`` and find the section

   .. code-block:: c++

      /* ======================
       * Fill Code: OMPL
       * ======================*/

   OMPL is a libarary containing several different planning algorithms. OMPL allows us to use as many different planners in parallel as we'd like until one has a result. For our implementation, we will choose to use only RRT Connect. 

   Below the above block, replace the current contents with the following

   .. code-block:: c++

      auto n = static_cast<Eigen::Index>(std::thread::hardware_concurrency());
      auto range = Eigen::VectorXd::LinSpaced(n, 0.005, 0.15);

   This implements the number of threads we will have planning in parallel. Now we can add as many planners as available threads.

   .. code-block:: c++

	auto profile = std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();
	profile->planning_time = 10.0;
	   profile->planners.reserve(static_cast<std::size_t>(n));
	   for (Eigen::Index i = 0; i < n; ++i)
	   {
	     auto rrt_connect = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
	     rrt_connect->range = range(i);
	     profile->planners.push_back(rrt_connect);
	   }
	return profile;

   There are many different OMPL planners available to experiment with. Feel free to play around with a few and observe how your application's motion plan changes (don't forget to include your chosen planner(s) in the header!).

#. Add the planner to the planning server:

   Return to ``planning_server.cpp`` and find the section where we add in our custom planning profiles.

   Copy and paste the follwing

   .. code-block:: c++

      profile_dict_->addProfile<tesseract_planning::OMPLPlanProfile>(
          tesseract_planning::profile_ns::OMPL_DEFAULT_NAMESPACE, PROFILE, createOMPLProfile());

   Now we have added our new OMPL planning profile to the planning server.

#. Add the planner to the taskflow:
   
   Go back to ``taskflow_generators.hpp``. Now we need to include our OMPL profile in our motion planning taskflow. Our freepsace taskflow will find a solution using OMPL and then improve that solution using TrajOpt. First, let's create our node for OMPL. Within ``FreespaceMotionPipelineTask`` add

   .. code:: c++

      // Setup OMPL
      boost::uuids::uuid ompl_planner_task =
         addNode(std::make_unique<tesseract_planning::OMPLMotionPlannerTask>(output_keys_[0], output_keys_[0]));
  
   Now we need to change our graph edges to incorporate these new nodes. Make the following changes to the edges:

   * ``min_length_task`` will connect to ``ompl_planner_task``

   * ``ompl_planner_task`` will connect to ``error_task`` and ``trajopt_planner_task``

   This taskflow now means OMPL will first find planning solutions and then TrajOpt will smooth out the trajectory.

   Now lets return to ``CartesianMotionPipelineTask`` and remove Descartes. Comment out where you created the Descartes node and where you connected ``descartes_planner_task`` to ``error_task`` and ``trajopt_planner_task``. Have ``min_length_task`` connect to ``trajopt_planner_task`` instead of your Descartes node. 

#. Run the application:

   Now try running the full application again with our completed motion planning pipeline. How has the plan changed since step one? Also take a look at our completed taskflow graph again and notice the new taskflow. Try playing around with changing some of the edges and see how the motion plan changes. Here are a few things you could try:

   * Remove TrajOpt and see how Descartes and OMPL perform without it.

   * Remove post-collision checking

      - You should also allow collisions in your planning profiles if you try this

   * Change the profiles of Descartes, OMPL, and TrajOpt

Congratulations! You have completed using Tesseract to create a motion plan for a "scan and plan" application!
