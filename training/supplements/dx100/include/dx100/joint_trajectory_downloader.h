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

#ifndef JOINT_TRAJECTORY_DOWNLOADER_H
#define JOINT_TRAJECTORY_DOWNLOADER_H

#include "simple_message/smpl_msg_connection.h"
#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectory.h>

namespace motoman
{
namespace joint_trajectory_downloader
{

/**
 * \brief Message handler that downloads joint trajectories to the
 * motoman controller
 */

//* JointTrajectoryDownloader
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointTrajectoryDownloader
{

public:

	JointTrajectoryDownloader();

  /**
   * \brief Constructor
   *
   * \param ROS node handle (used for subscribing)
   * \param ROS node handle (used for publishing (to the robot controller))
   */
	JointTrajectoryDownloader(ros::NodeHandle &n, industrial::smpl_msg_connection::SmplMsgConnection* robotConnecton);

  ~JointTrajectoryDownloader();

  void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);

private:

  industrial::smpl_msg_connection::SmplMsgConnection* robot_;
  ros::Subscriber sub_joint_trajectory_; //subscribe to "command"
  ros::NodeHandle node_;

  void trajectoryStop();

};

} //joint_trajectory_downloader
} //motoman

#endif /* JOINT_TRAJECTORY_DOWNLOADER_H */
