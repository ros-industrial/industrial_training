/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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

#include <dx100/joint_trajectory_handler.h>
#include "simple_message/messages/joint_message.h"
#include "simple_message/smpl_msg_connection.h"

using namespace industrial::smpl_msg_connection;
using namespace industrial::joint_message;
using namespace industrial::simple_message;

namespace motoman
{
namespace joint_trajectory_handler
{
JointTrajectoryHandler::JointTrajectoryHandler()
{
}

JointTrajectoryHandler::JointTrajectoryHandler(ros::NodeHandle &n, SmplMsgConnection* robotConnecton) :
    node_(n)
{
  ROS_INFO("Constructor joint trajectory handler node");

  this->mutex_.lock();
  this->sub_joint_tranectory_ = this->node_.subscribe("command", 0, &JointTrajectoryHandler::jointTrajectoryCB,
                                                      this);
  this->robot_ = robotConnecton;
  this->currentPoint = 0;
  this->state_ = JointTrajectoryStates::IDLE;
  this->trajectoryHandler_ =
      new boost::thread(boost::bind(&JointTrajectoryHandler::trajectoryHandler, this));
  ROS_INFO("Unlocking mutex");
  this->mutex_.unlock();
}

JointTrajectoryHandler::~JointTrajectoryHandler()
{  
  trajectoryStop();
  this->sub_joint_tranectory_.shutdown();
  delete this->trajectoryHandler_;
}

void JointTrajectoryHandler::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  ROS_INFO("Receiving joint trajctory message");
  this->mutex_.lock();
  ROS_INFO("Processing joint trajctory message (mutex acquired)");
  ROS_DEBUG("Current state is: %d", this->state_);
  if (JointTrajectoryStates::IDLE != this->state_)
  {
    if (msg->points.empty())
    {
      ROS_INFO("Empty trajectory received, canceling current trajectory");
    }
    else
    {
      ROS_ERROR("Trajectory splicing not yet implemented, stopping current motion.");
    }
    trajectoryStop();
  }
  if (msg->points.empty())
  {
    ROS_INFO("Empty trajectory received while in IDLE state, nothing is done");
  }
  else
  {
    ROS_INFO("Loading trajectory, setting state to streaming");
    this->current_traj_ = *msg;
    
    // TODO: This section pads any trajectory below the minimum size with the same
    // end point.  This is required because the motoman side requires a minimum buffer
    // size before it start motion.
    while (current_traj_.points.size() <= 6)
    {
      ROS_DEBUG("Padding trajectory, current size: %d", current_traj_.points.size());
      current_traj_.points.push_back(current_traj_.points[current_traj_.points.size()-1]);
    };
    
    ROS_INFO("Executing trajectory of size: %d", this->current_traj_.points.size());
    this->currentPoint = 0;
    this->state_ = JointTrajectoryStates::STREAMING;
    streaming_start_ = ros::Time::now();
  }
  this->mutex_.unlock();
}

void JointTrajectoryHandler::trajectoryHandler()
{
  JointMessage jMsg;
  SimpleMessage msg;
  SimpleMessage reply;
  trajectory_msgs::JointTrajectoryPoint pt;
  ROS_INFO("Starting joint trajectory handler state");
  while (ros::ok())
  {
    this->mutex_.lock();

    if (this->robot_->isConnected())
    {
      
      //TODO: These variables should be moved outside of this loop
      //so that we aren't contantly reinitializing them.
      JointMessage jMsg;
      SimpleMessage msg;
      SimpleMessage reply;
      trajectory_msgs::JointTrajectoryPoint pt;
      switch (this->state_)
      {
        case JointTrajectoryStates::IDLE:
          ros::Duration(0.250).sleep();
          break;


        case JointTrajectoryStates::STREAMING:
          if (this->currentPoint < this->current_traj_.points.size())
          {
            // unsigned int lastCurrentPoint = currentPoint;
            // currentPoint = getNextTrajectoryPoint(current_traj_,
            //                                       streaming_start_,
            //                                       ros::Time::now());
            //ROS_INFO("Skipping from point[%d] to point[%d]", lastCurrentPoint, currentPoint);
            pt = this->current_traj_.points[this->currentPoint];
            
            jMsg.setSequence(this->currentPoint);

            for (int i = 0; i < this->current_traj_.joint_names.size(); i++)
            {
              jMsg.getJoints().setJoint(i, pt.positions[i]);
            }
            
            ROS_DEBUG("Sending joint point");
            jMsg.toRequest(msg);
            if (this->robot_->sendAndReceiveMsg(msg, reply, false))
            //jMsg.toTopic(msg);
            //if (this->robot_->sendMsg(msg))
            {
              ROS_INFO("Point[%d of %d] sent to controller", 
                       this->currentPoint, this->current_traj_.points.size());
              this->currentPoint++;
            }
            else
            {
              ROS_WARN("Failed sent joint point, will try again");
            }
            //ROS_INFO_STREAM("Time taken to stream single point is " << (ros::WallTime::now()-start));
          }
          else
          {
            ROS_INFO("Trajectory streaming complete, setting state to IDLE");
            this->state_ = JointTrajectoryStates::IDLE;
          }
          break;

        default:
          ROS_ERROR("Joint trajectory handler: unknown state");
          this->state_ = JointTrajectoryStates::IDLE;
          break;
      }

    }
    else
    {
      ROS_INFO("Connecting to robot motion server");
      this->robot_->makeConnect();
    }

    this->mutex_.unlock();
    ros::Duration(0.005).sleep();
  }

  ROS_WARN("Exiting trajectory handler thread");
}

unsigned int JointTrajectoryHandler::getNextTrajectoryPoint(const trajectory_msgs::JointTrajectory& traj,
                                                            const ros::Time& start,
                                                            const ros::Time& cur)
{
  for(unsigned int i = 0; i < traj.points.size(); i++) {
    ros::Duration dur(cur-start);
    if(dur < traj.points[i].time_from_start) {
      return i;
    }
  }
  return traj.points.size()-1;
}


void JointTrajectoryHandler::trajectoryStop()
{
  JointMessage jMsg;
  SimpleMessage msg;
  SimpleMessage reply;

  ROS_INFO("Joint trajectory handler: entering stopping state");
  jMsg.setSequence(SpecialSeqValues::STOP_TRAJECTORY);
  jMsg.toRequest(msg);
  ROS_DEBUG("Sending stop command");
  this->robot_->sendAndReceiveMsg(msg, reply);
  ROS_DEBUG("Stop command sent, entring idle mode");
  this->state_ = JointTrajectoryStates::IDLE;
}





} //joint_trajectory_handler
} //motoman

