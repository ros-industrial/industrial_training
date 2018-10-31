# Introduction to Trajopt
## What is TrajOpt?
Trajopt is an optimization based path planner that utilizes Sequential Quadratic Programming. It models the path planning problem as an optimization problem of the costs and constraints provided by the user, then iteratively attempts to converge to a global minimum (though with no guarantee).


## Why use TrajOpt?

1. ***Speed***: Trajopt solves many problems very quickly.

2. ***Reliability***: Trajopt can reliably solve a large number of planning problems.
3. ***Flexibility***: Trajopt provides the ability for a user to define their own costs and constraints. This means that you can define a unique optimization objection specific to any application.


## Basic Usage

The typical workflow of using Trajopt for pathplanning is as follows.
1) Create a ProblemConstructionInfo. This can either be loaded from a json file or generated directly in C++. This contains information about the costs/constraints desired, the robot, and the planning environment.
2) Convert ProblemConstructionInfo to a TrajOptProb using ConstructProblem. This constructs the optimization into a standardized format used by the optimizers.
3) Optimize. The optimizer will optimize a `num_timesteps x num_joints + 1` matrix to minimize the costs and constraints. Each column represents a joint with the exception of the last one which is time.

### 1. Basic Info

Basic information for the planner

* ***[int] n_steps***:
        The number of trajectory states (timesteps) to be used in the planning of the trajectory.

* ***[string] manip***:
        The name of the manipulator of the robot to plan for.
        
* ***[bool] start_fixed***:
        Whether to force the first trajectory state to exactly equal the first state given in the init info.
        
* ***[string] robot (optional)***:
        TODO
* ***[vector/list of ints] dofs_fixed (optional)***:
        Indices corresponding to any degrees of freedom that you want to have remain in the same position the entire trajectory.

### 2. Init Info

Information detailing what initial trajectory Trajopt should start from.

* ***[InitInfo::Type] type***:
        The type of initialization. These only do anything if you are constructing the problem from Json, but you can still assign it if you feel like it. Valid values are:
        
    * ***STATIONARY***: Initializes the entire trajectory to the current joint states of the robot. No data is needed.
    * ***GIVEN_TRAJ***:
        You provide the entire initial trajectory in the **data** member
*    ***STRAIGHT_LINE***:
        You provide an **endpoint** member for the initial trajectory. The trajectory is the joint interpolated between the current state and the endpoint.

* ***[TrajArray] data***:
        Array containing the initialization information.
            - If doing C++, must contain a trajectory with all joints over all timesteps.
            - If using Json, you should only provide what is needed given the type you chose.

### 3. Optimization Info (optional)

These can typically be set to defaults. More details can be found by exploring the TrajOpt documentation

### 4. Costs

These are functions that you desire to be minimized, such as joint accelerations or cartesian velocities, but that you do not need the value to be driven to ~0. The weights of these terms will never increase.

### 5. Constraints

These are the functions that are needed to be driven to ~0. Failure to satisfy these constraints will cause the weighting of these terms to increase until they are satisfied or the maximum number of iterations is reached. Section 3.6 will cover the costs and contraints available.

## Plan


1. Use the relevant method to construct the problem

2. Optimize

3. View results

## Performance

There are several things that you can do to help speed the trajopt optimization.

1) Be sure to build your project in Release mode. Using the command line this is `catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release"` In Qt Creator, the build type option is under the projects tab.

2) Set the Trajopt logging level higher than Info before optimization. Do this with 
`#include <trajopt_utils/logging.hpp>
trajopt::gLogLevel = util::LevelWarn;'`

3) Change optimization parameters. The parameters in BasicTrustRegionSQPParameters can effect both the speed with which the optimization converges and the point at which the optimization triggers as having converged.

Notes
-----

At its core, trajopt does nothing more than take some cost functions and minimize them, but there are some details that the user should be aware of.

In updating the trajectory, trajopt treats both costs and constraints as costs. If it fails to satisfy constraints, then it will increase the weight (penalty) of the constraints in an effort to have them overcome the weight of the costs to move the trajectory toward satisfying the constraints. It will increase the penalty applied to constraint violations a finite number of times, so if the weights of your costs are too high, it may fail produce a result that satisfies constraints.

Furthermore, it should be noted that the result will be organized into a m x n matrix, where m is the number of timesteps and n is the number of joints + a column for time. The ith row then represents the joint state at the ith timestep of the trajectory.






