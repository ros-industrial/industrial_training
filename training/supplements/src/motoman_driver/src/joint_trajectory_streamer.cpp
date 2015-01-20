/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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
 *  * Neither the name of the Southwest Research Institute, nor the names
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

#include "motoman_driver/joint_trajectory_streamer.h"
#include "motoman_driver/simple_message/motoman_motion_reply_message.h"
#include "simple_message/messages/joint_traj_pt_full_message.h"
#include "motoman_driver/simple_message/messages/joint_traj_pt_full_ex_message.h"
#include "industrial_robot_client/utils.h"
#include "industrial_utils/param_utils.h"
#include <map>
#include <vector>
#include <string>

namespace CommTypes = industrial::simple_message::CommTypes;
namespace ReplyTypes = industrial::simple_message::ReplyTypes;
using industrial::joint_data::JointData;
using industrial::joint_traj_pt_full::JointTrajPtFull;
using industrial::joint_traj_pt_full_message::JointTrajPtFullMessage;
using industrial::joint_traj_pt_full_ex::JointTrajPtFullEx;
using industrial::joint_traj_pt_full_ex_message::JointTrajPtFullExMessage;

using motoman::simple_message::motion_reply_message::MotionReplyMessage;
namespace TransferStates = industrial_robot_client::joint_trajectory_streamer::TransferStates;
namespace MotionReplyResults = motoman::simple_message::motion_reply::MotionReplyResults;

namespace motoman
{
namespace joint_trajectory_streamer
{

#define ROS_ERROR_RETURN(rtn,...) do {ROS_ERROR(__VA_ARGS__); return(rtn);} while(0)

// override init() to read "robot_id" parameter and subscribe to joint_states
bool MotomanJointTrajectoryStreamer::init(SmplMsgConnection* connection, const std::map<int, RobotGroup> &robot_groups,
    const std::map<std::string, double> &velocity_limits)
{
  bool rtn = true;

  ROS_INFO("MotomanJointTrajectoryStreamer: init");

  this->robot_groups_ = robot_groups;
  rtn &= JointTrajectoryStreamer::init(connection, robot_groups, velocity_limits);

  motion_ctrl_.init(connection, 0);
  for (int i = 0; i < robot_groups_.size(); i++)
  {
    MotomanMotionCtrl motion_ctrl;

    int robot_id = robot_groups_[i].get_group_id();
    rtn &= motion_ctrl.init(connection, robot_id);

    motion_ctrl_map_[robot_id] = motion_ctrl;
  }
  return rtn;
}

// override init() to read "robot_id" parameter and subscribe to joint_states
bool MotomanJointTrajectoryStreamer::init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
    const std::map<std::string, double> &velocity_limits)
{
  bool rtn = true;

  ROS_INFO("MotomanJointTrajectoryStreamer: init");

  rtn &= JointTrajectoryStreamer::init(connection, joint_names, velocity_limits);

  // try to read robot_id parameter, if none specified
  if ((robot_id_ < 0))
    node_.param("robot_id", robot_id_, 0);

  rtn &= motion_ctrl_.init(connection, robot_id_);

  return rtn;
}

MotomanJointTrajectoryStreamer::~MotomanJointTrajectoryStreamer()
{
  //TODO Find better place to call StopTrajMode
  motion_ctrl_.setTrajMode(false);   // release TrajMode, so INFORM jobs can run
}

// override create_message to generate JointTrajPtFull message (instead of default JointTrajPt)
bool MotomanJointTrajectoryStreamer::create_message(int seq, const trajectory_msgs::JointTrajectoryPoint &pt, SimpleMessage *msg)
{
  JointTrajPtFull msg_data;
  JointData values;

  // copy position data
  if (!pt.positions.empty())
  {
    if (VectorToJointData(pt.positions, values))
      msg_data.setPositions(values);
    else
      ROS_ERROR_RETURN(false, "Failed to copy position data to JointTrajPtFullMessage");
  }
  else
    msg_data.clearPositions();

  // copy velocity data
  if (!pt.velocities.empty())
  {
    if (VectorToJointData(pt.velocities, values))
      msg_data.setVelocities(values);
    else
      ROS_ERROR_RETURN(false, "Failed to copy velocity data to JointTrajPtFullMessage");
  }
  else
    msg_data.clearVelocities();

  // copy acceleration data
  if (!pt.accelerations.empty())
  {
    if (VectorToJointData(pt.accelerations, values))
      msg_data.setAccelerations(values);
    else
      ROS_ERROR_RETURN(false, "Failed to copy acceleration data to JointTrajPtFullMessage");
  }
  else
    msg_data.clearAccelerations();

  // copy scalar data
  msg_data.setRobotID(robot_id_);
  msg_data.setSequence(seq);
  msg_data.setTime(pt.time_from_start.toSec());


  // convert to message
  JointTrajPtFullMessage jtpf_msg;
  jtpf_msg.init(msg_data);

  return jtpf_msg.toRequest(*msg);  // assume "request" COMM_TYPE for now
}

bool MotomanJointTrajectoryStreamer::create_message_ex(int seq, const motoman_msgs::DynamicJointPoint &point, SimpleMessage *msg)
{
  JointTrajPtFullEx msg_data_ex;
  JointTrajPtFullExMessage jtpf_msg_ex;
  std::vector<industrial::joint_traj_pt_full::JointTrajPtFull> msg_data_vector;

  JointData values;

  int num_groups = point.num_groups;

  for (int i = 0; i < num_groups; i++)
  {
    JointTrajPtFull msg_data;

    motoman_msgs::DynamicJointsGroup pt;

    motoman_msgs::DynamicJointPoint dpoint;

    pt = point.groups[i];

    if (pt.positions.size() < 10)
    {
      int size_to_complete = 10 - pt.positions.size();

      std::vector<double> positions(size_to_complete, 0.0);
      std::vector<double> velocities(size_to_complete, 0.0);
      std::vector<double> accelerations(size_to_complete, 0.0);

      pt.positions.insert(pt.positions.end(), positions.begin(), positions.end());
      pt.velocities.insert(pt.velocities.end(), velocities.begin(), velocities.end());
      pt.accelerations.insert(pt.accelerations.end(), accelerations.begin(), accelerations.end());
    }

    // copy position data
    if (!pt.positions.empty())
    {
      if (VectorToJointData(pt.positions, values))
        msg_data.setPositions(values);
      else
        ROS_ERROR_RETURN(false, "Failed to copy position data to JointTrajPtFullMessage");
    }
    else
      msg_data.clearPositions();
    // copy velocity data
    if (!pt.velocities.empty())
    {
      if (VectorToJointData(pt.velocities, values))
        msg_data.setVelocities(values);
      else
        ROS_ERROR_RETURN(false, "Failed to copy velocity data to JointTrajPtFullMessage");
    }
    else
      msg_data.clearVelocities();

    // copy acceleration data
    if (!pt.accelerations.empty())
    {
      if (VectorToJointData(pt.accelerations, values))
        msg_data.setAccelerations(values);
      else
        ROS_ERROR_RETURN(false, "Failed to copy acceleration data to JointTrajPtFullMessage");
    }
    else
      msg_data.clearAccelerations();

    // copy scalar data
    msg_data.setRobotID(pt.group_number);
    msg_data.setSequence(seq);
    msg_data.setTime(pt.time_from_start.toSec());

    // convert to message
    msg_data_vector.push_back(msg_data);
  }

  msg_data_ex.setMultiJointTrajPtData(msg_data_vector);
  msg_data_ex.setNumGroups(num_groups);
  msg_data_ex.setSequence(seq);
  jtpf_msg_ex.init(msg_data_ex);

  return jtpf_msg_ex.toRequest(*msg);  // assume "request" COMM_TYPE for now
}

bool MotomanJointTrajectoryStreamer::create_message(int seq, const motoman_msgs::DynamicJointsGroup &pt, SimpleMessage *msg)
{
  JointTrajPtFull msg_data;
  JointData values;
  // copy position data
  if (!pt.positions.empty())
  {
    if (VectorToJointData(pt.positions, values))
      msg_data.setPositions(values);
    else
      ROS_ERROR_RETURN(false, "Failed to copy position data to JointTrajPtFullMessage");
  }
  else
    msg_data.clearPositions();

  // copy velocity data
  if (!pt.velocities.empty())
  {
    if (VectorToJointData(pt.velocities, values))
      msg_data.setVelocities(values);
    else
      ROS_ERROR_RETURN(false, "Failed to copy velocity data to JointTrajPtFullMessage");
  }
  else
    msg_data.clearVelocities();

  // copy acceleration data
  if (!pt.accelerations.empty())
  {
    if (VectorToJointData(pt.accelerations, values))
      msg_data.setAccelerations(values);
    else
      ROS_ERROR_RETURN(false, "Failed to copy acceleration data to JointTrajPtFullMessage");
  }
  else
    msg_data.clearAccelerations();

  // copy scalar data
  msg_data.setRobotID(pt.group_number);

  msg_data.setSequence(seq);
  msg_data.setTime(pt.time_from_start.toSec());

  // convert to message
  JointTrajPtFullMessage jtpf_msg;
  jtpf_msg.init(msg_data);

  return jtpf_msg.toRequest(*msg);  // assume "request" COMM_TYPE for now
}

bool MotomanJointTrajectoryStreamer::VectorToJointData(const std::vector<double> &vec,
    JointData &joints)
{
  if (vec.size() > joints.getMaxNumJoints())
    ROS_ERROR_RETURN(false, "Failed to copy to JointData.  Len (%d) out of range (0 to %d)",
                     vec.size(), joints.getMaxNumJoints());

  joints.init();
  for (int i = 0; i < vec.size(); ++i)
  {
    joints.setJoint(i, vec[i]);
  }
  return true;
}

// override send_to_robot to provide controllerReady() and setTrajMode() calls
bool MotomanJointTrajectoryStreamer::send_to_robot(const std::vector<SimpleMessage>& messages)
{
  if (!motion_ctrl_.controllerReady() && !motion_ctrl_.setTrajMode(true))
    ROS_ERROR_RETURN(false, "Failed to initialize MotoRos motion.  Trajectory ABORTED.  Correct issue and re-send trajectory.");

  return JointTrajectoryStreamer::send_to_robot(messages);
}

// override streamingThread, to provide check/retry of MotionReply.result=BUSY
void MotomanJointTrajectoryStreamer::streamingThread()
{
  int connectRetryCount = 1;

  ROS_INFO("Starting Motoman joint trajectory streamer thread");
  while (ros::ok())
  {
    ros::Duration(0.005).sleep();

    // automatically re-establish connection, if required
    if (connectRetryCount-- > 0)
    {
      ROS_INFO("Connecting to robot motion server");
      this->connection_->makeConnect();
      ros::Duration(0.250).sleep();  // wait for connection

      if (this->connection_->isConnected())
        connectRetryCount = 0;
      else if (connectRetryCount <= 0)
      {
        ROS_ERROR("Timeout connecting to robot controller.  Send new motion command to retry.");
        this->state_ = TransferStates::IDLE;
      }
      continue;
    }

    this->mutex_.lock();

    SimpleMessage msg, tmpMsg, reply;

    switch (this->state_)
    {
    case TransferStates::IDLE:
      ros::Duration(0.250).sleep();  //  slower loop while waiting for new trajectory
      break;

    case TransferStates::STREAMING:
      if (this->current_point_ >= static_cast<int>(this->current_traj_.size()))
      {
        ROS_INFO("Trajectory streaming complete, setting state to IDLE");
        this->state_ = TransferStates::IDLE;
        break;
      }

      if (!this->connection_->isConnected())
      {
        ROS_DEBUG("Robot disconnected.  Attempting reconnect...");
        connectRetryCount = 5;
        break;
      }

      tmpMsg = this->current_traj_[this->current_point_];
      msg.init(tmpMsg.getMessageType(), CommTypes::SERVICE_REQUEST,
               ReplyTypes::INVALID, tmpMsg.getData());  // set commType=REQUEST

      if (!this->connection_->sendAndReceiveMsg(msg, reply, false))
        ROS_WARN("Failed sent joint point, will try again");
      else
      {
        MotionReplyMessage reply_status;
        if (!reply_status.init(reply))
        {
          ROS_ERROR("Aborting trajectory: Unable to parse JointTrajectoryPoint reply");
          this->state_ = TransferStates::IDLE;
          break;
        }

        if (reply_status.reply_.getResult() == MotionReplyResults::SUCCESS)
        {
          ROS_DEBUG("Point[%d of %d] sent to controller",
                    this->current_point_, static_cast<int>(this->current_traj_.size()));
          this->current_point_++;
        }
        else if (reply_status.reply_.getResult() == MotionReplyResults::BUSY)
          break;  // silently retry sending this point
        else
        {
          ROS_ERROR_STREAM("Aborting Trajectory.  Failed to send point"
                           << " (#" << this->current_point_ << "): "
                           << MotomanMotionCtrl::getErrorString(reply_status.reply_));
          this->state_ = TransferStates::IDLE;
          break;
        }
      }
      break;
    default:
      ROS_ERROR("Joint trajectory streamer: unknown state");
      this->state_ = TransferStates::IDLE;
      break;
    }
    this->mutex_.unlock();
  }
  ROS_WARN("Exiting trajectory streamer thread");
}

// override trajectoryStop to send MotionCtrl message
void MotomanJointTrajectoryStreamer::trajectoryStop()
{
  this->state_ = TransferStates::IDLE;  // stop sending trajectory points
  motion_ctrl_.stopTrajectory();
}

// override is_valid to include FS100-specific checks
bool MotomanJointTrajectoryStreamer::is_valid(const trajectory_msgs::JointTrajectory &traj)
{
  if (!JointTrajectoryInterface::is_valid(traj))
    return false;

  for (int i = 0; i < traj.points.size(); ++i)
  {
    const trajectory_msgs::JointTrajectoryPoint &pt = traj.points[i];

    // FS100 requires valid velocity data
    if (pt.velocities.empty())
      ROS_ERROR_RETURN(false, "Validation failed: Missing velocity data for trajectory pt %d", i);
  }

  if ((cur_joint_pos_.header.stamp - ros::Time::now()).toSec() > pos_stale_time_)
    ROS_ERROR_RETURN(false, "Validation failed: Can't get current robot position.");

  // FS100 requires trajectory start at current position
  namespace IRC_utils = industrial_robot_client::utils;
  if (!IRC_utils::isWithinRange(cur_joint_pos_.name, cur_joint_pos_.position,
                                traj.joint_names, traj.points[0].positions,
                                start_pos_tol_))
  {
    ROS_ERROR_RETURN(false, "Validation failed: Trajectory doesn't start at current position.");
  }
  return true;
}

bool MotomanJointTrajectoryStreamer::is_valid(const motoman_msgs::DynamicJointTrajectory &traj)
{
  if (!JointTrajectoryInterface::is_valid(traj))
    return false;
  ros::Time time_stamp;
  int group_number;
  for (int i = 0; i < traj.points.size(); ++i)
  {
    for (int gr = 0; gr < traj.points[i].num_groups; gr++)
    {
      const motoman_msgs::DynamicJointsGroup &pt = traj.points[i].groups[gr];
      time_stamp = cur_joint_pos_map_[pt.group_number].header.stamp;
      //TODO: adjust for more joints
      group_number = pt.group_number;
      // FS100 requires valid velocity data
      if (pt.velocities.empty())
        ROS_ERROR_RETURN(false, "Validation failed: Missing velocity data for trajectory pt %d", i);

      // FS100 requires trajectory start at current position
      namespace IRC_utils = industrial_robot_client::utils;

      if (!IRC_utils::isWithinRange(cur_joint_pos_map_[group_number].name, cur_joint_pos_map_[group_number].position,
                                    traj.joint_names, traj.points[0].groups[gr].positions,
                                    start_pos_tol_))
      {
        ROS_ERROR_RETURN(false, "Validation failed: Trajectory doesn't start at current position.");
      }
    }
  }

  if ((time_stamp - ros::Time::now()).toSec() > pos_stale_time_)
    ROS_ERROR_RETURN(false, "Validation failed: Can't get current robot position.");

  return true;
}

}  // namespace joint_trajectory_streamer
}  // namespace motoman

