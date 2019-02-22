# Costs and Constraints


This section provides an overview of the common features of costs and constraints, the different types and their meaning, and a list of the currently avaible costs and constraints. Additional information can be found in the TrajOpt documentation as well as in the trajopt_examples package.

Common Ground
-------------

Costs and constraints are specific types of *TermInfos* in Trajopt. Many of the same types of terms can be used as either a cost or a constraint. This is set by setting the term_type member to either TT_COST or TT_CNT. When using the json interface, this is handled for you. Additionally, there is a TT_USE_TIME option that allows some terms to use time parameterization.

All terms also have an optional ```name``` parameter that you can set to specify a name for that specific term. When using Json, this defaults to the term type. In C++, name is empty by default.



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

When constructing the problem in C++, each term has a respective TermInfo object. Create a ```std::shared_ptr``` to the type of term you wish to add, update the parameter members of that object, and ```push_back``` that term to the end of ```cost_infos``` or ```cnt_infos``` of the ```ProblemConstructionInfo``` object. If the term can be either a cost or a constraint, do not forget to set the ```term_type``` member to the proper enumeration to reflect whether you intend the term to be a cost or a constraint.

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
    term->term_type = trajopt::TT_COST 
    pci.cost_infos.push_back(term);
    
    // or 
    
    // if constraint
    term->term_type = trajopt::TT_CNT
    pci.cnt_infos.push_back(term)
    
```



## Joint Level Terms

There are a number of joint level costs available in TrajOpt. These are 

- Joint Position
- Joint Velocity
- Joint Acceleration
- Joint Jerk

Some important notes:

- Each of these is calculated from the joint position values using numerical differentiation. If the trajectory is too finely discretized, these values may be somewhat meaningless
- When time parameterization is disabled, these costs assume unit time steps. This means velocity for instance is just `x1-x0`. This is a useful cost, but the user should be aware of what it means.
- Each of these costs have a target (required), an upper_tol (optional), and a lower_tol (optional). 
    - The target sets the desired value (usually 0) for all joints. Thus it should be a vector the length of the DOF.
    - If `upper_tol==lower_tol==0` then an equality costs/cnt will be set with a squared error.
    - If `upper_tol!=lower_tol` then a hinge cost/cnt will be set, centered about target and hinged at the tolerances. 
        - Note: This "dead band" of 0 cost can cause numerical issues sometimes. It is often advantageous to set a small equality cost in addition to a larger hinge cost if one is needed.

## Cartesian Terms

Similarly, there are a number of cartesian terms available in TrajOpt. They are:

- Dynamic Cartesian Pose (DynamicCartPoseTermInfo)
- Cartesian Pose (CartPoseTermInfo)
- Cartesian Velocity (CartVelTermInfo)

Note: These make use of the FK and numeric gradient calculation and come with a higher computation cost than joint level costs. However, this is not significant for most cases, and it is often possible to solve even hundreds of these at once (see the puzzle_piece example).

## Miscellaneous Terms

Miscellaneous terms include:

- Collision
 
The collision cost is one of TrajOpt's most notable features. Using features in Tesseract, it is able to use convex-convex collision checking to very quickly calculate collisions. In addition to discrete point collision checking. It also has the ability to do collision checking of a swept volume between two joint states. When an object is found in collision, it is able to provide a vector to get out of collision - allowing for more intelligent state updates between iterations.


