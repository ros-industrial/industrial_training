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

#ifndef JOINT_TRAJECTORY_HANDLER_H
#define JOINT_TRAJECTORY_HANDLER_H

#include "simple_message/smpl_msg_connection.h"
#include "ros/ros.h"
#include <boost/thread/thread.hpp>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace motoman
{
namespace joint_trajectory_handler
{

namespace JointTrajectoryStates
{
enum JointTrajectoryState
{
  IDLE = 0, STREAMING =1 //,STARTING, //, STOPPING
};
}
typedef JointTrajectoryStates::JointTrajectoryState JointTrajectoryState;

/**
 * \brief Message handler that relays joint trajectories to the motoman controller
 */

//* JointTrajectoryHandler
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointTrajectoryHandler
{

public:

  JointTrajectoryHandler();

  /**
   * \brief Constructor
   *
   * \param ROS node handle (used for subscribing)
   * \param ROS node handle (used for publishing (to the robot controller))
   */
  JointTrajectoryHandler(ros::NodeHandle &n, industrial::smpl_msg_connection::SmplMsgConnection* robotConnecton);

  ~JointTrajectoryHandler();

  void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);
  void trajectoryHandler();

  unsigned int getNextTrajectoryPoint(const trajectory_msgs::JointTrajectory& traj,
                                      const ros::Time& start,
                                      const ros::Time& cur);

private:

  void trajectoryStop();

  industrial::smpl_msg_connection::SmplMsgConnection* robot_;
  ros::Subscriber sub_joint_tranectory_; //subscribe to "command"
  ros::NodeHandle node_;

  boost::thread* trajectoryHandler_;
  boost::mutex mutex_;int currentPoint;
  trajectory_msgs::JointTrajectory current_traj_;
  JointTrajectoryState state_;
  ros::Time streaming_start_;

  static const int NUM_OF_JOINTS_ = 7;
};

} //joint_trajectory_handler
} //motoman

#endif /* JOINT_TRAJECTORY_HANDLER_H */
