# Optimization Based Planning Introduction

## Goal
 * The purpose of these exercises is to implement a ROS node that drives a robot through a series of moves and actions in order to complete a pick and place task.  In addition, they will serve as an example of how to utilize more specilized tools such as the optimization based path planner, TrajOpt, and the control interfaces of the Franka robot, while also integrating a variety of software capabilities (perception, controller drivers, I/O, inverse kinematics, path planning, collision avoidance, etc) into a ROS-based industrial application.

## Objectives
 * Understand the components and structure of a real or simulated robot application.
 * Learn how to setup a TrajOpt path planning problem
 * Learn how to use costs and constraints in TrajOpt
 * Learn how to move the arm to a joint or Cartesian position.
 * Leverage perception capabilities using PCL
 * Plan collision-free paths for a pick and place task
 * Control robot peripherals such as a gripper
