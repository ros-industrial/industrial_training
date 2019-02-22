# Introduction to Trajopt
## What is TrajOpt?
Trajopt is an optimization based path planner that utilizes Sequential Quadratic Programming. It models the path planning problem as an optimization problem of the costs and constraints provided by the user, then iteratively attempts to converge to a global minimum (though with no guarantee).


## Why use TrajOpt?

1. ***Speed***: Trajopt solves many problems very quickly.

2. ***Reliability***: Trajopt can reliably solve a large number of planning problems.
3. ***Flexibility***: Trajopt provides the ability for a user to define their own costs and constraints. This means that you can define a unique optimization objection specific to any application.


## Basic Usage

The typical workflow of using Trajopt for pathplanning is as follows.
1) Create a ProblemConstructionInfo (pci). This can either be loaded from a json file or generated directly in C++. This contains information about the costs/constraints desired, the robot, and the planning environment.
2) Convert ProblemConstructionInfo to a TrajOptProb using ConstructProblem. This constructs the optimization into a standardized format used by the optimizers.
3) Optimize. The optimizer will optimize a `num_timesteps x num_joints + 1` matrix to minimize the costs and constraints. Each column represents a joint with the exception of the last one which is time.

The next sections will cover some of the important parts of setting up a TrajOpt problem.

### 1. Basic Info

The basic_info section of the pci contains (as you might expect) basic information for the planner.

* ***[int] n_steps***:
        The number of trajectory states (timesteps) to be used in the planning of the trajectory.

* ***[string] manip***:
        The name of the manipulator of the robot to plan for.

* ***[string] robot (optional)***:
    TODO

* ***[bool] start_fixed***:
        Whether to force the first trajectory state to exactly equal the first state given in the init info. If true, the associated joint values will be set to the initial conditions with an equality constraint

* ***[vector/list of ints] dofs_fixed (optional)***:
        Indices corresponding to any degrees of freedom that you want to have remain in the same position the entire trajectory. These will be set to the initial conditions with an equality constraint

* ***[sco::ModelType] convex_solver (optional)***:
    This specifies the solver to use. If unspecified, the included BPMPD solver will be used.

* ***[bool] use_time ***:
    If true, TrajOpt adds a column to represent (1/dt). This must be set to true if any of the costs/constraints are set to use time.

* ***[double] dt_upper_limit (optional)***:
    This is the upper limit of 1/dt values allowed. Note this value is 1/dt ***NOT*** dt.

* ***[double] dt_lower_limit (optional)***:
    This is the lower limit of 1/dt values allowed. Note this value is 1/dt ***NOT*** dt.

### 2. Init Info

The init_info section of the pci contains information detailing the initial trajectory Trajopt should start from.

* ***[InitInfo::Type] type***:
        The type of initialization. Valid values are:
        
    * ***STATIONARY***: Initializes the entire trajectory to the current joint states of the robot. No data is needed.
    * ***GIVEN_TRAJ***:
        You provide the entire initial trajectory in the **data** member
*    ***JOINT_INTERPOLATED***:
        You provide an **endpoint** member for the initial trajectory. The trajectory is the joint interpolated between the current state and the endpoint.

* ***[TrajArray] data***:
        Array containing the initialization information.
            - If doing C++, must contain a trajectory with all joints over all timesteps.
            - If using Json, you should only provide what is needed given the type you chose.

### 3. Optimization Info (optional)

This section of the pci can typically be set to defaults. In fact, when using a Tesseract planner as we will in this demo, the selections here get overwritten. These are the parameters that govern the SQP optimization routine. More details can be found by exploring the TrajOpt documentation.

### 4. Costs

These are functions that you desire to be minimized, such as joint accelerations or cartesian velocities. The optimizer will seek to optimize the sum of the weighted costs subject to the constraints below. These correspond to the objective function. These will be discussed in more detail in 3.6.

### 5. Constraints

These are the conditions that must be satisfied by the optimizer. Failure to satisfy these constraints will cause the weighting of these terms to increase until they are satisfied or the maximum number of iterations is reached. These directly correspond to optimizer constraints. Section 3.6 will cover the costs and contraints available.


## Performance

There are several things that you can do to help speed the trajopt optimization.

1) Be sure to build your project in Release mode. Using the command line this is `catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release"` In Qt Creator, the build type option is under the projects tab.

2) Set the Trajopt logging level higher than Info before optimization. Do this with `#include <trajopt_utils/logging.hpp>
trajopt::gLogLevel = util::LevelWarn;`

3) Change optimization parameters. The parameters in BasicTrustRegionSQPParameters can effect both the speed with which the optimization converges and the point at which the optimization triggers as having converged.

4) Change your problem. This may seem obvious, but sometimes constraints can be mutually exlusive. You can also have two very similar competing costs (this is one of TrajOpt's advantages). For example, a cartesian pose constraint that is out of the robot workspace would cause a failure. A cartesian pose cost that is in collision could slow down optimization. Be judicious about which terms are really necessary to be constraints and which ones could be costs.

Notes
-----

At its core, trajopt does nothing more than take some cost functions and minimize them, but there are some details that the user should be aware of.

In updating the trajectory, trajopt treats both costs and constraints as costs. If it fails to satisfy constraints, then it will increase the weight (penalty) of the constraints in an effort to have them overcome the weight of the costs to move the trajectory toward satisfying the constraints. It will increase the penalty applied to constraint violations a finite number of times, so if the weights of your costs are too high, it may fail produce a result that satisfies constraints.

Furthermore, it should be noted that the result will be organized into a m x n matrix, where m is the number of timesteps and n is the number of joints + a column for time. The ith row then represents the joint state at the ith timestep of the trajectory.






