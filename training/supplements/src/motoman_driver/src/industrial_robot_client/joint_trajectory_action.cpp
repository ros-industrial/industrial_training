/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Fraunhofer IPA
 * Author: Thiago de Freitas
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Fraunhofer IPA, nor the names
 *  of its contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
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

#include <motoman_driver/industrial_robot_client/joint_trajectory_action.h>
#include <industrial_robot_client/utils.h>
#include <industrial_utils/param_utils.h>
#include <industrial_utils/utils.h>
#include <map>
#include <string>
#include <vector>

namespace industrial_robot_client
{
namespace joint_trajectory_action
{

const double JointTrajectoryAction::WATCHD0G_PERIOD_ = 1.0;
const double JointTrajectoryAction::DEFAULT_GOAL_THRESHOLD_ = 0.01;

JointTrajectoryAction::JointTrajectoryAction() :
  action_server_(node_, "joint_trajectory_action",
                 boost::bind(&JointTrajectoryAction::goalCB, this, _1),
                 boost::bind(&JointTrajectoryAction::cancelCB, this, _1), false)
{
  ros::NodeHandle pn("~");

  pn.param("constraints/goal_threshold", goal_threshold_, DEFAULT_GOAL_THRESHOLD_);

  std::map<int, RobotGroup> robot_groups;

  // This loops through the yaml file to find the parameters for correctly configuring the node
  // A parameter named topics_list contains the respective information for each of the groups

  std::string value;
  ros::param::search("topics_list", value);

  XmlRpc::XmlRpcValue topics_list_rpc;
  ros::param::get(value, topics_list_rpc);


  std::vector<XmlRpc::XmlRpcValue> topics_list;

  for (int i = 0; i < topics_list_rpc.size(); i++)
  {
    XmlRpc::XmlRpcValue state_value;
    state_value = topics_list_rpc[i];
    topics_list.push_back(state_value);
  }

  std::vector<XmlRpc::XmlRpcValue> groups_list;
  // TODO(thiagodefreitas): check the consistency of the group numbers
  for (int i = 0; i < topics_list[0]["state"].size(); i++)
  {
    XmlRpc::XmlRpcValue group_value;
    group_value = topics_list[0]["state"][i];
    groups_list.push_back(group_value);
  }


  for (int i = 0; i < groups_list.size(); i++)
  {
    RobotGroup rg;
    std::vector<std::string> rg_joint_names;

    XmlRpc::XmlRpcValue joints;

    joints = groups_list[i]["group"][0]["joints"];
    for (int jt = 0; jt < joints.size(); jt++)
      rg_joint_names.push_back(static_cast<std::string>(joints[jt]));

    XmlRpc::XmlRpcValue group_number;


    group_number = groups_list[i]["group"][0]["group_number"];
    int group_number_int = static_cast<int>(group_number);

    XmlRpc::XmlRpcValue name;
    std::string name_string;

    name = groups_list[i]["group"][0]["name"];
    name_string = static_cast<std::string>(name);


    XmlRpc::XmlRpcValue ns;
    std::string ns_string;

    ns = groups_list[i]["group"][0]["ns"];

    ns_string = static_cast<std::string>(ns);

    rg.set_group_id(group_number_int);
    rg.set_joint_names(rg_joint_names);
    rg.set_name(name_string);
    rg.set_ns(ns_string);

    std::string joint_path_action_name = ns_string + "/" + name_string;
    robot_groups[group_number_int] = rg;

    all_joint_names_.insert(all_joint_names_.end(), rg_joint_names.begin(), rg_joint_names.end());

    actionServer_ = new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>(
      node_, joint_path_action_name + "/joint_trajectory_action" , false);
    actionServer_->registerGoalCallback(
      boost::bind(&JointTrajectoryAction::goalCB,
                  this, _1, group_number_int));
    actionServer_->registerCancelCallback(
      boost::bind(&JointTrajectoryAction::cancelCB,
                  this, _1, group_number_int));

    pub_trajectory_command_ = node_.advertise<motoman_msgs::DynamicJointTrajectory>(
                                joint_path_action_name + "/joint_path_command", 1);
    sub_trajectory_state_  = this->node_.subscribe<control_msgs::FollowJointTrajectoryFeedback>(
                               joint_path_action_name + "/feedback_states", 1,
                               boost::bind(&JointTrajectoryAction::controllerStateCB,
                                           this, _1, group_number_int));
    sub_robot_status_ = node_.subscribe(
                          "robot_status", 1, &JointTrajectoryAction::robotStatusCB, this);

    pub_trajectories_[group_number_int] = pub_trajectory_command_;
    sub_trajectories_[group_number_int] = (sub_trajectory_state_);
    sub_status_[group_number_int] = (sub_robot_status_);

    this->act_servers_[group_number_int] = actionServer_;

    this->act_servers_[group_number_int]->start();

    this->watchdog_timer_map_[group_number_int] = node_.createTimer(
          ros::Duration(WATCHD0G_PERIOD_), boost::bind(
            &JointTrajectoryAction::watchdog, this, _1, group_number_int));
  }

  pub_trajectory_command_ = node_.advertise<motoman_msgs::DynamicJointTrajectory>(
                              "joint_path_command", 1);

  this->robot_groups_ = robot_groups;

  action_server_.start();
}

JointTrajectoryAction::~JointTrajectoryAction()
{
}

void JointTrajectoryAction::robotStatusCB(
  const industrial_msgs::RobotStatusConstPtr &msg)
{
  last_robot_status_ = msg;  // caching robot status for later use.
}

void JointTrajectoryAction::watchdog(const ros::TimerEvent &e)
{
  // Some debug logging
  if (!last_trajectory_state_)
  {
    ROS_DEBUG("Waiting for subscription to joint trajectory state");
  }
  if (!trajectory_state_recvd_)
  {
    ROS_DEBUG("Trajectory state not received since last watchdog");
  }

  // Aborts the active goal if the controller does not appear to be active.
  if (has_active_goal_)
  {
    if (!trajectory_state_recvd_)
    {
      // last_trajectory_state_ is null if the subscriber never makes a connection
      if (!last_trajectory_state_)
      {
        ROS_WARN("Aborting goal because we have never heard a controller state message.");
      }
      else
      {
        ROS_WARN_STREAM(
          "Aborting goal because we haven't heard from the controller in " << WATCHD0G_PERIOD_ << " seconds");
      }
      abortGoal();
    }
  }

  // Reset the trajectory state received flag
  trajectory_state_recvd_ = false;
}

void JointTrajectoryAction::watchdog(const ros::TimerEvent &e, int group_number)
{
  // Some debug logging
  if (!last_trajectory_state_map_[group_number])
  {
    ROS_DEBUG("Waiting for subscription to joint trajectory state");
  }
  if (!trajectory_state_recvd_map_[group_number])
  {
    ROS_DEBUG("Trajectory state not received since last watchdog");
  }

  // Aborts the active goal if the controller does not appear to be active.
  if (has_active_goal_map_[group_number])
  {
    if (!trajectory_state_recvd_map_[group_number])
    {
      // last_trajectory_state_ is null if the subscriber never makes a connection
      if (!last_trajectory_state_map_[group_number])
      {
        ROS_WARN("Aborting goal because we have never heard a controller state message.");
      }
      else
      {
        ROS_WARN_STREAM(
          "Aborting goal because we haven't heard from the controller in " << WATCHD0G_PERIOD_ << " seconds");
      }
      abortGoal(group_number);
    }
  }
  // Reset the trajectory state received flag
  trajectory_state_recvd_ = false;
}

void JointTrajectoryAction::goalCB(JointTractoryActionServer::GoalHandle & gh)
{
  gh.setAccepted();

  int group_number;

// TODO(thiagodefreitas): change for getting the id from the group instead of a sequential checking on the map

  ros::Duration last_time_from_start(0.0);

  motoman_msgs::DynamicJointTrajectory dyn_traj;

  for (int i = 0; i < gh.getGoal()->trajectory.points.size(); i++)
  {
    motoman_msgs::DynamicJointPoint dpoint;

    for (int rbt_idx = 0; rbt_idx < robot_groups_.size(); rbt_idx++)
    {
      size_t ros_idx = std::find(
                         gh.getGoal()->trajectory.joint_names.begin(),
                         gh.getGoal()->trajectory.joint_names.end(),
                         robot_groups_[rbt_idx].get_joint_names()[0])
                       - gh.getGoal()->trajectory.joint_names.begin();

      bool is_found = ros_idx < gh.getGoal()->trajectory.joint_names.size();

      group_number = rbt_idx;
      motoman_msgs::DynamicJointsGroup dyn_group;

      int num_joints = robot_groups_[group_number].get_joint_names().size();

      if (is_found)
      {
        if (gh.getGoal()->trajectory.points[i].positions.empty())
        {
          std::vector<double> positions(num_joints, 0.0);
          dyn_group.positions = positions;
        }
        else
          dyn_group.positions.insert(
            dyn_group.positions.begin(),
            gh.getGoal()->trajectory.points[i].positions.begin() + ros_idx,
            gh.getGoal()->trajectory.points[i].positions.begin() + ros_idx
            + robot_groups_[rbt_idx].get_joint_names().size());

        if (gh.getGoal()->trajectory.points[i].velocities.empty())
        {
          std::vector<double> velocities(num_joints, 0.0);
          dyn_group.velocities = velocities;
        }
        else
          dyn_group.velocities.insert(
            dyn_group.velocities.begin(),
            gh.getGoal()->trajectory.points[i].velocities.begin()
            + ros_idx, gh.getGoal()->trajectory.points[i].velocities.begin()
            + ros_idx + robot_groups_[rbt_idx].get_joint_names().size());

        if (gh.getGoal()->trajectory.points[i].accelerations.empty())
        {
          std::vector<double> accelerations(num_joints, 0.0);
          dyn_group.accelerations = accelerations;
        }
        else
          dyn_group.accelerations.insert(
            dyn_group.accelerations.begin(),
            gh.getGoal()->trajectory.points[i].accelerations.begin()
            + ros_idx, gh.getGoal()->trajectory.points[i].accelerations.begin()
            + ros_idx + robot_groups_[rbt_idx].get_joint_names().size());
        if (gh.getGoal()->trajectory.points[i].effort.empty())
        {
          std::vector<double> effort(num_joints, 0.0);
          dyn_group.effort = effort;
        }
        else
          dyn_group.effort.insert(
            dyn_group.effort.begin(),
            gh.getGoal()->trajectory.points[i].effort.begin()
            + ros_idx, gh.getGoal()->trajectory.points[i].effort.begin()
            + ros_idx + robot_groups_[rbt_idx].get_joint_names().size());
        dyn_group.time_from_start = gh.getGoal()->trajectory.points[i].time_from_start;
        dyn_group.group_number = group_number;
        dyn_group.num_joints = dyn_group.positions.size();
      }

      // Generating message for groups that were not present in the trajectory message
      else
      {
        std::vector<double> positions(num_joints, 0.0);
        std::vector<double> velocities(num_joints, 0.0);
        std::vector<double> accelerations(num_joints, 0.0);
        std::vector<double> effort(num_joints, 0.0);

        dyn_group.positions = positions;
        dyn_group.velocities = velocities;
        dyn_group.accelerations = accelerations;
        dyn_group.effort = effort;

        dyn_group.time_from_start = gh.getGoal()->trajectory.points[i].time_from_start;
        dyn_group.group_number = group_number;
        dyn_group.num_joints = num_joints;
      }

      dpoint.groups.push_back(dyn_group);
    }
    dpoint.num_groups = dpoint.groups.size();
    dyn_traj.points.push_back(dpoint);
  }
  dyn_traj.header = gh.getGoal()->trajectory.header;
  dyn_traj.header.stamp = ros::Time::now();
  // Publishing the joint names for the 4 groups
  dyn_traj.joint_names = all_joint_names_;

  this->pub_trajectory_command_.publish(dyn_traj);
}

void JointTrajectoryAction::cancelCB(JointTractoryActionServer::GoalHandle & gh)
{
  // The interface is provided, but it is recommended to use
  //  void JointTrajectoryAction::cancelCB(JointTractoryActionServer::GoalHandle & gh, int group_number)

  ROS_DEBUG("Received action cancel request");
}

void JointTrajectoryAction::goalCB(JointTractoryActionServer::GoalHandle & gh, int group_number)
{
  if (!gh.getGoal()->trajectory.points.empty())
  {
    if (industrial_utils::isSimilar(
          this->robot_groups_[group_number].get_joint_names(),
          gh.getGoal()->trajectory.joint_names))
    {
      // Cancels the currently active goal.
      if (has_active_goal_map_[group_number])
      {
        ROS_WARN("Received new goal, canceling current goal");
        abortGoal(group_number);
      }
      // Sends the trajectory along to the controller
      if (withinGoalConstraints(last_trajectory_state_map_[group_number],
                                gh.getGoal()->trajectory, group_number))
      {
        ROS_INFO_STREAM("Already within goal constraints, setting goal succeeded");
        gh.setAccepted();
        gh.setSucceeded();
        has_active_goal_map_[group_number] = false;
      }
      else
      {
        gh.setAccepted();
        active_goal_map_[group_number] = gh;
        has_active_goal_map_[group_number]  = true;

        ROS_INFO("Publishing trajectory");

        current_traj_map_[group_number] = active_goal_map_[group_number].getGoal()->trajectory;

        int num_joints = robot_groups_[group_number].get_joint_names().size();

        motoman_msgs::DynamicJointTrajectory dyn_traj;

        for (int i = 0; i < current_traj_map_[group_number].points.size(); ++i)
        {
          motoman_msgs::DynamicJointsGroup dyn_group;
          motoman_msgs::DynamicJointPoint dyn_point;

          if (gh.getGoal()->trajectory.points[i].positions.empty())
          {
            std::vector<double> positions(num_joints, 0.0);
            dyn_group.positions = positions;
          }
          else
            dyn_group.positions = gh.getGoal()->trajectory.points[i].positions;

          if (gh.getGoal()->trajectory.points[i].velocities.empty())
          {
            std::vector<double> velocities(num_joints, 0.0);
            dyn_group.velocities = velocities;
          }
          else
            dyn_group.velocities = gh.getGoal()->trajectory.points[i].velocities;
          if (gh.getGoal()->trajectory.points[i].accelerations.empty())
          {
            std::vector<double> accelerations(num_joints, 0.0);
            dyn_group.accelerations = accelerations;
          }
          else
            dyn_group.accelerations = gh.getGoal()->trajectory.points[i].accelerations;
          if (gh.getGoal()->trajectory.points[i].effort.empty())
          {
            std::vector<double> effort(num_joints, 0.0);
            dyn_group.effort = effort;
          }
          else
            dyn_group.effort = gh.getGoal()->trajectory.points[i].effort;
          dyn_group.time_from_start = gh.getGoal()->trajectory.points[i].time_from_start;
          dyn_group.group_number = group_number;
          dyn_group.num_joints = robot_groups_[group_number].get_joint_names().size();
          dyn_point.groups.push_back(dyn_group);

          dyn_point.num_groups = 1;
          dyn_traj.points.push_back(dyn_point);
        }
        dyn_traj.header = gh.getGoal()->trajectory.header;
        dyn_traj.joint_names = gh.getGoal()->trajectory.joint_names;
        this->pub_trajectories_[group_number].publish(dyn_traj);
      }
    }
    else
    {
      ROS_ERROR("Joint trajectory action failing on invalid joints");
      control_msgs::FollowJointTrajectoryResult rslt;
      rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(rslt, "Joint names do not match");
    }
  }
  else
  {
    ROS_ERROR("Joint trajectory action failed on empty trajectory");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Empty trajectory");
  }

  // Adding some informational log messages to indicate unsupported goal constraints
  if (gh.getGoal()->goal_time_tolerance.toSec() > 0.0)
  {
    ROS_WARN_STREAM("Ignoring goal time tolerance in action goal, may be supported in the future");
  }
  if (!gh.getGoal()->goal_tolerance.empty())
  {
    ROS_WARN_STREAM(
      "Ignoring goal tolerance in action, using paramater tolerance of " << goal_threshold_ << " instead");
  }
  if (!gh.getGoal()->path_tolerance.empty())
  {
    ROS_WARN_STREAM("Ignoring goal path tolerance, option not supported by ROS-Industrial drivers");
  }
}

void JointTrajectoryAction::cancelCB(
  JointTractoryActionServer::GoalHandle & gh, int group_number)
{
  ROS_DEBUG("Received action cancel request");
  if (active_goal_map_[group_number] == gh)
  {
    // Stops the controller.
    motoman_msgs::DynamicJointTrajectory empty;
    empty.joint_names = robot_groups_[group_number].get_joint_names();
    this->pub_trajectories_[group_number].publish(empty);

    // Marks the current goal as canceled.
    active_goal_map_[group_number].setCanceled();
    has_active_goal_map_[group_number] = false;
  }
  else
  {
    ROS_WARN("Active goal and goal cancel do not match, ignoring cancel request");
  }
}

void JointTrajectoryAction::controllerStateCB(
  const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg, int robot_id)
{
  ROS_DEBUG("Checking controller state feedback");
  last_trajectory_state_map_[robot_id] = msg;
  trajectory_state_recvd_map_[robot_id] = true;

  if (!has_active_goal_map_[robot_id])
  {
    ROS_DEBUG("No active goal, ignoring feedback");
    return;
  }

  if (current_traj_map_[robot_id].points.empty())
  {
    ROS_DEBUG("Current trajectory is empty, ignoring feedback");
    return;
  }

  if (!industrial_utils::isSimilar(robot_groups_[robot_id].get_joint_names(), msg->joint_names))
  {
    ROS_ERROR("Joint names from the controller don't match our joint names.");
    return;
  }

  // Checking for goal constraints
  // Checks that we have ended inside the goal constraints and has motion stopped

  ROS_DEBUG("Checking goal constraints");
  if (withinGoalConstraints(last_trajectory_state_map_[robot_id], current_traj_map_[robot_id], robot_id))
  {
    if (last_robot_status_)
    {
      // Additional check for motion stoppage since the controller goal may still
      // be moving.  The current robot driver calls a motion stop if it receives
      // a new trajectory while it is still moving.  If the driver is not publishing
      // the motion state (i.e. old driver), this will still work, but it warns you.
      if (last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
      {
        ROS_INFO("Inside goal constraints, stopped moving, return success for action");
        active_goal_map_[robot_id].setSucceeded();
        has_active_goal_map_[robot_id] = false;
      }
      else if (last_robot_status_->in_motion.val == industrial_msgs::TriState::UNKNOWN)
      {
        ROS_INFO("Inside goal constraints, return success for action");
        ROS_WARN("Robot status in motion unknown, the robot driver node and controller code should be updated");
        active_goal_map_[robot_id].setSucceeded();
        has_active_goal_map_[robot_id] = false;
      }
      else
      {
        ROS_DEBUG("Within goal constraints but robot is still moving");
      }
    }
    else
    {
      ROS_INFO("Inside goal constraints, return success for action");
      ROS_WARN("Robot status is not being published the robot driver node and controller code should be updated");
      active_goal_map_[robot_id].setSucceeded();
      has_active_goal_map_[robot_id] = false;
    }
  }
}

void JointTrajectoryAction::controllerStateCB(
  const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
{
  ROS_DEBUG("Checking controller state feedback");
  last_trajectory_state_ = msg;
  trajectory_state_recvd_ = true;

  if (!has_active_goal_)
  {
    ROS_DEBUG("No active goal, ignoring feedback");
    return;
  }
  if (current_traj_.points.empty())
  {
    ROS_DEBUG("Current trajectory is empty, ignoring feedback");
    return;
  }

  if (!industrial_utils::isSimilar(all_joint_names_, msg->joint_names))
  {
    ROS_ERROR("Joint names from the controller don't match our joint names.");
    return;
  }

  // Checking for goal constraints
  // Checks that we have ended inside the goal constraints and has motion stopped

  ROS_DEBUG("Checking goal constraints");
  if (withinGoalConstraints(last_trajectory_state_, current_traj_))
  {
    if (last_robot_status_)
    {
      // Additional check for motion stoppage since the controller goal may still
      // be moving.  The current robot driver calls a motion stop if it receives
      // a new trajectory while it is still moving.  If the driver is not publishing
      // the motion state (i.e. old driver), this will still work, but it warns you.
      if (last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
      {
        ROS_INFO("Inside goal constraints, stopped moving, return success for action");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
      else if (last_robot_status_->in_motion.val == industrial_msgs::TriState::UNKNOWN)
      {
        ROS_INFO("Inside goal constraints, return success for action");
        ROS_WARN("Robot status in motion unknown, the robot driver node and controller code should be updated");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
      else
      {
        ROS_DEBUG("Within goal constraints but robot is still moving");
      }
    }
    else
    {
      ROS_INFO("Inside goal constraints, return success for action");
      ROS_WARN("Robot status is not being published the robot driver node and controller code should be updated");
      active_goal_.setSucceeded();
      has_active_goal_ = false;
    }
  }
}

void JointTrajectoryAction::abortGoal()
{
  // Stops the controller.
  trajectory_msgs::JointTrajectory empty;
  pub_trajectory_command_.publish(empty);

  // Marks the current goal as aborted.
  active_goal_.setAborted();
  has_active_goal_ = false;
}

void JointTrajectoryAction::abortGoal(int robot_id)
{
  // Stops the controller.
  motoman_msgs::DynamicJointTrajectory empty;
  pub_trajectories_[robot_id].publish(empty);

  // Marks the current goal as aborted.
  active_goal_map_[robot_id].setAborted();
  has_active_goal_map_[robot_id] = false;
}

bool JointTrajectoryAction::withinGoalConstraints(
  const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
  const trajectory_msgs::JointTrajectory & traj)
{
  bool rtn = false;
  if (traj.points.empty())
  {
    ROS_WARN("Empty joint trajectory passed to check goal constraints, return false");
    rtn = false;
  }
  else
  {
    int last_point = traj.points.size() - 1;

    if (industrial_robot_client::utils::isWithinRange(
          last_trajectory_state_->joint_names,
          last_trajectory_state_->actual.positions, traj.joint_names,
          traj.points[last_point].positions, goal_threshold_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
    }
  }
  return rtn;
}

bool JointTrajectoryAction::withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
    const motoman_msgs::DynamicJointTrajectory & traj)
{
  bool rtn = false;
  if (traj.points.empty())
  {
    ROS_WARN("Empty joint trajectory passed to check goal constraints, return false");
    rtn = false;
  }
  else
  {
    int last_point = traj.points.size() - 1;

    const motoman_msgs::DynamicJointsGroup &pt = traj.points[last_point].groups[3];

    int group_number = pt.group_number;

    if (industrial_robot_client::utils::isWithinRange(
          last_trajectory_state_map_[group_number]->joint_names,
          last_trajectory_state_map_[group_number]->actual.positions, traj.joint_names,
          traj.points[last_point].groups[3].positions, goal_threshold_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
    }
  }
  return rtn;
}

bool JointTrajectoryAction::withinGoalConstraints(
  const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
  const trajectory_msgs::JointTrajectory & traj, int robot_id)
{
  bool rtn = false;
  if (traj.points.empty())
  {
    ROS_WARN("Empty joint trajectory passed to check goal constraints, return false");
    rtn = false;
  }
  else
  {
    int last_point = traj.points.size() - 1;

    int group_number = robot_id;

    if (industrial_robot_client::utils::isWithinRange(
          robot_groups_[group_number].get_joint_names(),
          last_trajectory_state_map_[group_number]->actual.positions, traj.joint_names,
          traj.points[last_point].positions, goal_threshold_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
    }
  }
  return rtn;
}

}  // namespace joint_trajectory_action
}  // namespace industrial_robot_client


