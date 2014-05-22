/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <industrial_msgs/RobotStatus.h>

const double DEFAULT_GOAL_THRESHOLD = 0.01;

class JointTrajectoryExecuter
{
private:
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
  typedef JTAS::GoalHandle GoalHandle;
public:
  JointTrajectoryExecuter(ros::NodeHandle &n) :
      node_(n), action_server_(node_, "joint_trajectory_action",
                               boost::bind(&JointTrajectoryExecuter::goalCB, this, _1),
                               boost::bind(&JointTrajectoryExecuter::cancelCB, this, _1), false), has_active_goal_(
          false)
  {
    using namespace XmlRpc;
    ros::NodeHandle pn("~");

    joint_names_.push_back("joint_s");
    joint_names_.push_back("joint_l");
    joint_names_.push_back("joint_e");
    joint_names_.push_back("joint_u");
    joint_names_.push_back("joint_r");
    joint_names_.push_back("joint_b");
    joint_names_.push_back("joint_t");

    pn.param("constraints/goal_time", goal_time_constraint_, 0.0);

    // Gets the constraints for each joint.
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      std::string ns = std::string("constraints/") + joint_names_[i];
      double g, t;
      pn.param(ns + "/goal", g, DEFAULT_GOAL_THRESHOLD);
      pn.param(ns + "/trajectory", t, -1.0);
      goal_constraints_[joint_names_[i]] = g;
      trajectory_constraints_[joint_names_[i]] = t;
    }
    pn.param("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_, 0.01);

    pub_controller_command_ = node_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
    sub_controller_state_ = node_.subscribe("feedback_states", 1, &JointTrajectoryExecuter::controllerStateCB, this);
    sub_robot_status_ = node_.subscribe("robot_status", 1, &JointTrajectoryExecuter::robotStatusCB, this);

    action_server_.start();
  }

  ~JointTrajectoryExecuter()
  {
    pub_controller_command_.shutdown();
    sub_controller_state_.shutdown();
    watchdog_timer_.stop();
  }

  void robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg)
  {
    last_robot_status_ = *msg;  //caching robot status for later use.
  }

  bool withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
                             const std::map<std::string, double>& constraints,
                             const trajectory_msgs::JointTrajectory& traj)
  {
    ROS_DEBUG("Checking goal contraints");
    int last = traj.points.size() - 1;
    for (size_t i = 0; i < msg->joint_names.size(); ++i)
    {
      double abs_error = fabs(msg->actual.positions[i] - traj.points[last].positions[i]);
      double goal_constraint = constraints.at(msg->joint_names[i]);
      if (goal_constraint >= 0 && abs_error > goal_constraint)
      {
        ROS_DEBUG("Bad constraint: %f, abs_errs: %f", goal_constraint, abs_error);
        return false;
      }
      ROS_DEBUG("Checking constraint: %f, abs_errs: %f", goal_constraint, abs_error);
    }
    return true;
  }

private:

  static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
  {
    if (a.size() != b.size())
      return false;

    for (size_t i = 0; i < a.size(); ++i)
    {
      if (count(b.begin(), b.end(), a[i]) != 1)
        return false;
    }
    for (size_t i = 0; i < b.size(); ++i)
    {
      if (count(a.begin(), a.end(), b[i]) != 1)
        return false;
    }

    return true;
  }

  void watchdog(const ros::TimerEvent &e)
  {
    ros::Time now = ros::Time::now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
      bool should_abort = false;
      if (!last_controller_state_)
      {
        should_abort = true;
        ROS_WARN("Aborting goal because we have never heard a controller state message.");
      }
      else if ((now - last_controller_state_->header.stamp) > ros::Duration(5.0))
      {
        should_abort = true;
        ROS_WARN(
            "Aborting goal because we haven't heard from the controller in %.3lf seconds", (now - last_controller_state_->header.stamp).toSec());
      }

      if (should_abort)
      {
        // Stops the controller.
        trajectory_msgs::JointTrajectory empty;
        empty.joint_names = joint_names_;
        pub_controller_command_.publish(empty);

        // Marks the current goal as aborted.
        active_goal_.setAborted();
        has_active_goal_ = false;
      }
    }
  }

  void goalCB(GoalHandle gh)
  {
    // Ensures that the joints in the goal match the joints we are commanding.
    ROS_DEBUG("Received goal: goalCB");
    if (!setsEqual(joint_names_, gh.getGoal()->trajectory.joint_names))
    {
      ROS_ERROR("Joints on incoming goal don't match our joints");
      gh.setRejected();
      return;
    }

    // Cancels the currently active goal.
    if (has_active_goal_)
    {
      ROS_DEBUG("Received new goal, canceling current goal");
      // Stops the controller.
      trajectory_msgs::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }

    // Sends the trajectory along to the controller
    if (withinGoalConstraints(last_controller_state_, goal_constraints_, gh.getGoal()->trajectory))
    {
      ROS_INFO_STREAM("Already within goal constraints");
      gh.setAccepted();
      gh.setSucceeded();
    }
    else
    {
      gh.setAccepted();
      active_goal_ = gh;
      has_active_goal_ = true;

      ROS_INFO("Publishing trajectory");

      current_traj_ = active_goal_.getGoal()->trajectory;
      pub_controller_command_.publish(current_traj_);
    }
  }

  void cancelCB(GoalHandle gh)
  {
    ROS_DEBUG("Received action cancel request");
    if (active_goal_ == gh)
    {
      // Stops the controller.
      trajectory_msgs::JointTrajectory empty;
      empty.joint_names = joint_names_;
      pub_controller_command_.publish(empty);

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
  }

  ros::NodeHandle node_;
  JTAS action_server_;
  ros::Publisher pub_controller_command_;
  ros::Subscriber sub_controller_state_;
  ros::Subscriber sub_robot_status_;
  ros::Timer watchdog_timer_;

  bool has_active_goal_;
  GoalHandle active_goal_;
  trajectory_msgs::JointTrajectory current_traj_;

  std::vector<std::string> joint_names_;
  std::map<std::string, double> goal_constraints_;
  std::map<std::string, double> trajectory_constraints_;
  double goal_time_constraint_;
  double stopped_velocity_tolerance_;

  control_msgs::FollowJointTrajectoryFeedbackConstPtr last_controller_state_;
  industrial_msgs::RobotStatus last_robot_status_;

  void controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
  {
    //ROS_DEBUG("Checking controller state feedback");
    last_controller_state_ = msg;
    ros::Time now = ros::Time::now();

    if (!has_active_goal_)
    {
      //ROS_DEBUG("No active goal, ignoring feedback");
      return;
    }
    if (current_traj_.points.empty())
    {
      ROS_DEBUG("Current trajectory is empty, ignoring feedback");
      return;
    }
    /* NOT CONCERNED ABOUT TRAJECTORY TIMING AT THIS POINT
     if (now < current_traj_.header.stamp + current_traj_.points[0].time_from_start)
     return;
     */

    if (!setsEqual(joint_names_, msg->joint_names))
    {
      ROS_ERROR("Joint names from the controller don't match our joint names.");
      return;
    }

    // Checking for goal constraints
    // Checks that we have ended inside the goal constraints and has motion stopped

    ROS_DEBUG("Checking goal constraints");
    if (withinGoalConstraints(msg, goal_constraints_, current_traj_))
    {
      // Additional check for motion stoppage since the controller goal may still
      // be moving.  The current robot driver calls a motion stop if it receives
      // a new trajectory while it is still moving.  If the driver is not publishing
      // the motion state (i.e. old driver), this will still work, but it warns you.
      if (last_robot_status_.in_motion.val == industrial_msgs::TriState::FALSE)
      {
        ROS_INFO("Inside goal constraints, stopped moving, return success for action");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
      else if (last_robot_status_.in_motion.val == industrial_msgs::TriState::UNKNOWN)
      {
        ROS_INFO("Inside goal constraints, return success for action");
        ROS_WARN("Robot status: in motion unknown, the robot driver node and controller code should be updated");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
      else
      {
        ROS_DEBUG("Within goal constraints but robot is still moving");
      }
    }
    // Verifies that the controller has stayed within the trajectory constraints.
    /*  DISABLING THIS MORE COMPLICATED GOAL CHECKING AND ERROR DETECTION


     int last = current_traj_.points.size() - 1;
     ros::Time end_time = current_traj_.header.stamp + current_traj_.points[last].time_from_start;
     if (now < end_time)
     {
     // Checks that the controller is inside the trajectory constraints.
     for (size_t i = 0; i < msg->joint_names.size(); ++i)
     {
     double abs_error = fabs(msg->error.positions[i]);
     double constraint = trajectory_constraints_[msg->joint_names[i]];
     if (constraint >= 0 && abs_error > constraint)
     {
     // Stops the controller.
     trajectory_msgs::JointTrajectory empty;
     empty.joint_names = joint_names_;
     pub_controller_command_.publish(empty);

     active_goal_.setAborted();
     has_active_goal_ = false;
     ROS_WARN("Aborting because we would up outside the trajectory constraints");
     return;
     }
     }
     }
     else
     {
     // Checks that we have ended inside the goal constraints
     bool inside_goal_constraints = true;
     for (size_t i = 0; i < msg->joint_names.size() && inside_goal_constraints; ++i)
     {
     double abs_error = fabs(msg->error.positions[i]);
     double goal_constraint = goal_constraints_[msg->joint_names[i]];
     if (goal_constraint >= 0 && abs_error > goal_constraint)
     inside_goal_constraints = false;

     // It's important to be stopped if that's desired.
     if (fabs(msg->desired.velocities[i]) < 1e-6)
     {
     if (fabs(msg->actual.velocities[i]) > stopped_velocity_tolerance_)
     inside_goal_constraints = false;
     }
     }

     if (inside_goal_constraints)
     {
     active_goal_.setSucceeded();
     has_active_goal_ = false;
     }
     else if (now < end_time + ros::Duration(goal_time_constraint_))
     {
     // Still have some time left to make it.
     }
     else
     {
     ROS_WARN("Aborting because we wound up outside the goal constraints");
     active_goal_.setAborted();
     has_active_goal_ = false;
     }

     }
     */
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_trajectory_action_node");
  ros::NodeHandle node; //("~");
  JointTrajectoryExecuter jte(node);

  ros::spin();

  return 0;
}

