# Costs and Constraints


This section provides an overview of the common features of costs and constraints, the different types and their meaning, and a list of the currently avaible costs and constraints, as well as what information to provide them in order to fully define them.

Common Ground
-------------

Costs and constraints are specific types of *terms* in Trajopt. Many of the same types of terms can be used as either a cost or a constraint. All terms also have an optional ```name``` parameter that you can set to specify a name for that specific term. When using Json, this defaults to the term type. In C++, name is empty by default.

Adding Terms with Json
----------------------

When constructing the problem using Json, it will go in a list of costs or constraints. You then need to specificy the type of term (Joint Velocity, etc) and a member :code:`params` that contains all of the parameters for that type of term.

``` json

    {
    "costs" : 
    [
    OR
    {
    "constraints" :
    [
    ...
    
        {
            "type" : "this_term_type",
            "name" : "optional_term_name",
            "params" :
            {
                "string_a" : "string",
                "int_b" : 1,
                "float_c" : 0.5
            }
        }
    ]
    }

```
Adding Terms with C++
---------------------

When constructing the problem in C++, each term has a respective Type(Cost/Constraint/Term)Info object. Create a ```std::shared_ptr``` to the type of term you wish to add, update the parameter members of that object, and ```push_back``` that term to the end of ```cost_infos``` or ```cnt_infos``` of the ```ProblemConstructionInfo``` object. If the term can be either a cost or a constraint, do not forget to set the ```term_type``` member to the proper enumeration to reflect whether you intend the term to be a cost or a constraint.

``` cpp

    // make the term
    std::shared_ptr<TypeTermInfo> term(new TypeTermInfo);
    
    // set the parameters
    term->string_a = "string";
    term->int_b = 1;
    term->float_c = 0.5;
    
    // optionally set its name
    term->name = "optional_term_name";
    
    // push_back onto the correct infos vector
    // pci = ProblemConstructionInfo
    
    // if cost
    term->term_type = trajopt::TT_COST // if needed
    pci.cost_infos.push_back(term);
    
    // or 
    
    // if constraint
    term->term_type = trajopt::TT_CNT // if needed
    pci.cnt_infos.push_back(term)
    
```

Term Dictionary
---------------

Below are all of the curently available terms, descriptions of them, and all of the parameters that must be provided to define them. The format for the entry will be as follows.

    - Type *[C++ TermInfo Object Type/Json type]*
        Description
        
        [data type] Member1
            Description
        [data type] Member2
            Description
        [data type] Member3
            Description

## Costs


    - Joint Position Cost *[JointPosCostInfo/"joint_pos"]*
        Cost value proportional to the squared difference between the joint positions at a timestep and the values provided
        
        [std::vector<double>] vals
            Values of joint positions at which the cost is zero
        [std::vector<double>] coeffs
            Coefficients multiplied by the squared differences of the joint positions and values. Effectively weights for each joint in this cost.
            
    - Joint Velcoity Cost *[JointVelCostInfo/"joint_vel"]*
        Cost value proportional to the squared velocities of the trajectory as computed from numerical differentiation
        
        [std::vector<double>] coeffs
            Coefficients multiplied by the squared velocities. Effectively weights for the velocity of each joint in this cost.

    - Joint Acceleration Cost *[JointAccCostInfo/"joint_acc"]*
        Cost value proportional to the squared accelerations of the trajectory as computed from numerical differentiation
        
        [std::vector<double>] coeffs
            Coefficients multiplied by the squared accelerations. Effectively weights for the acceleartion of each joint in this cost.
            
    - Joint Jerk Cost *[JointJerkCostInfo/"joint_jerk"]*
        Cost value proportional to the squared jerks of the trajectory as computed from numerical differentiation
        
        [std::vector<double>] coeffs
            Coefficients multiplied by the squared jerks. Effectively weights for the jerk of each joint in this cost.


## Constraints


    - Joint Position Constraint *[JointConstraintInfo/"joint"]*
        Constraint on the joint positions of the robot at a given timestep.
        
        [int] timestep
            The timestep of the trajectory to apply the constraint to
        [std::vector<double>] vals
            The joint positions to constrain the joints to.
            
    - Joint Velocity Constraint *[JointVelConstraintInfo/"joint_vel_limits"]*
        Constraint on the displacement in joint space for all joints between each set of consecutive time step between a starting and ending time step.
        
        [int] first_step
            The starting time step to apply the constraint to
        [int] last_step
            The final time step to apply the constraint to
        [std::vector<double>] vals
            The displacement limits for all of the joints. The nth joint will be given the limit in the nth index of **vals**

    - Cartesian Velocity Constraint *[CartVelCntInfo/"cart_vel"]*
        Constraint on the displacement in Cartesian space between each set of consecutive time step between a starting and ending timestep. NOTE: this constraint only applies to the individual displacement components and not to the resultant displacement.
        
        [int] first_step
            The starting time step to apply the constraint to
        [int] last_step
            The final time step to apply the constraint to
        [std::string] link
            The link whose velocity in Cartesian space is being constrained
        [double] max_displacement
            The maximum displacement in a any cardinal direction between two consecutive timesteps
        

## Costs/Constraints



    - Pose *[PoseCostInfo/"pose"]*
        Term for the pose of a link relative to another link
        
        [int] timestep
            The timestep of the trajectory to apply the term to
        [string] target
            Link with the pose desired
        [Eigen::Vector3d] pos_coefffs
            Weighting coefficients for the x, y, and z translational error respectively
        [Eigen::Vector3d] rot_coefffs
            Weighting coefficients for the x, y, and z components of the orientation quaternion
        [std::string] link
            Link that is desired to be in the same pose as **target**
        [Eigen::Affine3d] tcp
            Tool center point relative to **link**
            
    - Static Pose *[StaticPoseCostInfo/"static_pose"]*
        Term for the pose of a link in fixed space
        
        [int] timestep
            The timestep of the trajectory to apply the term to
        [Eigen::Vector3d] xyz
            x, y, and z coordinates of the desired pose
        [Eigen::Vector4d] wxyz
            w, x, y, and z components of the quaternion 
        [Eigen::Vector3d] pos_coefffs
            Weighting coefficients for the x, y, and z translational error respectively
        [Eigen::Vector3d] rot_coefffs
            Weighting coefficients for the x, y, and z components of the orientation quaternion
        [std::string] link
            The link that is desired to be in the pose provided
        [Eigen::Affine3d] tcp
            Tool center point relative to **link**
            
    - Collision *[CollisionCostInfo/"collision"]*
        Collision checking term for the trajectory between a starting and ending timestep
        
        [int] first_step
            The starting time step to apply the constraint to
        [int] last_step
            The final time step to apply the constraint to
        [bool] continuous
            Whether to use continuous collision checking or not. Continuous checking is done with swept volumes between time steps.
        [int] gap
            The number of timesteps between steps that you wish to check for collision. For example, if gap = 2 and the first_step = 1, then collision will be checked at timestep 1, then timestep 3, then teimstep 5, etc. (with swept volumes between those steps if using continuous)
        [vector<SafetyMarginDataPtr>] info
            List of items containing distance penalization data, such as the safety margin, weighting coefficient, etc. This can be easily constructed with the :code:`createSafetyMarginDataVector` method.
