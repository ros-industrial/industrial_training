/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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

#ifndef FS100_JOINT_TRAJECTORY_STREAMER_H
#define FS100_JOINT_TRAJECTORY_STREAMER_H

#include "fs100/fs100_motion_ctrl.h"
#include "fs100/industrial_robot_client/joint_trajectory_streamer.h"
#include "simple_message/joint_data.h"
#include "simple_message/simple_message.h"

namespace motoman
{
namespace fs100_joint_trajectory_streamer
{

using motoman::fs100_motion_ctrl::FS100_MotionCtrl;
using industrial_robot_client::joint_trajectory_streamer::JointTrajectoryStreamer;
using industrial::simple_message::SimpleMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;

/**
 * \brief Message handler that streams joint trajectories to the robot controller.
 *        Contains FS100-specific motion control commands.
 */

//* JointTrajectoryStreamer
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class FS100_JointTrajectoryStreamer : public JointTrajectoryStreamer
{

public:

  // since this class overrides some base-class methods,
  // these statements help find the base-class versions
  using JointTrajectoryStreamer::init;
  using JointTrajectoryInterface::is_valid;

  /**
   * \brief Default constructor
   *
   * \param robot_id robot group # on this controller (for multi-group systems)
   */
  FS100_JointTrajectoryStreamer(int robot_id=-1) : JointTrajectoryStreamer(1),
                                                  robot_id_(robot_id) {}

  ~FS100_JointTrajectoryStreamer();

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send commands to robot (ALREADY INITIALIZED)
   * \param joint_names list of expected joint-names.
   *   - Count and order should match data sent to robot connection.
   *   - Use blank-name to insert a placeholder joint position (typ. 0.0).
   *   - Joints in the incoming JointTrajectory stream that are NOT listed here will be ignored.
   * \param velocity_limits map of maximum velocities for each joint
   *   - leave empty to lookup from URDF
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                    const std::map<std::string, double> &velocity_limits = std::map<std::string, double>());

  /**
   * \brief Create SimpleMessage for sending to the robot
   *
   * \param[in] seq sequence # of this point in the overall trajectory
   * \param[in] pt  trajectory point data
   * \param[out] msg message for sending to robot
   *
   * \return true on success, false otherwise
   */
  virtual bool create_message(int seq, const trajectory_msgs::JointTrajectoryPoint &pt, SimpleMessage* msg);

  virtual bool send_to_robot(const std::vector<SimpleMessage>& messages);

  virtual void streamingThread();

protected:
  static const double pos_stale_time_ = 1.0;  // max time since last "current position" update, for validation (sec)
  static const double start_pos_tol_  = 1e-4; // max difference btwn start & current position, for validation (rad)

  int robot_id_;
  FS100_MotionCtrl motion_ctrl_;

  void trajectoryStop();
  bool is_valid(const trajectory_msgs::JointTrajectory &traj);

  static bool VectorToJointData(const std::vector<double> &vec,
                                industrial::joint_data::JointData &joints);
};

} //fs100_joint_trajectory_streamer
} //motoman

#endif /* FS100_JOINT_TRAJECTORY_STREAMER_H */
