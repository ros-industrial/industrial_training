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

#ifndef MOTOMAN_DRIVER_SIMPLE_MESSAGE_JOINT_TRAJ_PT_FULL_EX_H
#define MOTOMAN_DRIVER_SIMPLE_MESSAGE_JOINT_TRAJ_PT_FULL_EX_H

#ifndef FLATHEADERS
#include "simple_message/joint_data.h"
#include "simple_message/simple_message.h"
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/joint_traj_pt_full.h"
#else
#include "joint_data.h"
#include "simple_message.h"
#include "simple_serialize.h"
#include "shared_types.h"
#endif

#include<vector>
namespace industrial
{
namespace joint_traj_pt_full_ex
{

namespace SpecialSeqValues
{
enum SpecialSeqValue
{
  START_TRAJECTORY_DOWNLOAD = -1, START_TRAJECOTRY_STREAMING = -2, END_TRAJECTORY = -3, STOP_TRAJECTORY = -4
};
}
typedef SpecialSeqValues::SpecialSeqValue SpecialSeqValue;

/**
 * \brief Class encapsulated joint trajectory point data.  The point data
 * serves as a waypoint along a trajectory and is meant to mirror the
 * JointTrajectoryPoint message.
 *
 * This class is similar to the simple_message joint_traj_pt class, but this
 * class provides the full message contents directly to the robot controller,
 * rather than simplifying the velocity duration.
 *
 * The message data-packet byte representation is as follows (ordered lowest index
 * to highest). The standard sizes are given, but can change based on type sizes:
 *
 *   member:             type                                      size
 *   num_groups          (industrial::shared_types::shared_int)    4  bytes
 *   sequence            (industrial::shared_types::shared_int)    4  bytes
 *   multiJointTrajData  JointTrajPtData[]
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class JointTrajPtFullEx : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  JointTrajPtFullEx(void);
  /**
   * \brief Destructor
   *
   */
  ~JointTrajPtFullEx(void);

  /**
   * \brief Initializes a empty joint trajectory group
   *
   */
  void init();

  /**
   * \brief Initializes a complete trajectory group
   *
   */
  void init(industrial::shared_types::shared_int num_groups,
            industrial::shared_types::shared_int sequence,
            std::vector<industrial::joint_traj_pt_full::JointTrajPtFull> joint_trajectory_points);

  /**
   * \brief Sets num_groups
   *        Number of groups attached to the controller
   *
   * \param num_groups new num_groups value
   */
  void setNumGroups(industrial::shared_types::shared_int num_groups)
  {
    this->num_groups_ = num_groups;
  }

  /**
   * \brief Sets groups_number_
   *        Numbers of group, this sets the amount of control groups connected to the controller
   *
   * \param groups_number new groups_number value
   */

  industrial::shared_types::shared_int getNumGroups()
  {
    return this->num_groups_;
  }

  void setMultiJointTrajPtData(std::vector<industrial::joint_traj_pt_full::JointTrajPtFull> joint_trajectory_points)
  {
    this->joint_trajectory_points_ = joint_trajectory_points;
  }

  /**
   * \brief Sets joint trajectory point sequence number
   *
   * \param sequence value
   */
  void setSequence(industrial::shared_types::shared_int sequence)
  {
    this->sequence_ = sequence;
  }

  /**
   * \brief Returns joint trajectory point maximum number of groups
   *
   * \return joint trajectory maximum number of groups
   */

  industrial::shared_types::shared_int getMaxGroups()
  {
    return MAX_NUM_GROUPS;
  }

  /**
   * \brief Returns joint trajectory point sequence number
   *
   * \return joint trajectory sequence number
   */

  industrial::shared_types::shared_int getSequence()
  {
    return this->sequence_;
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(JointTrajPtFullEx &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(JointTrajPtFullEx &rhs);


  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return sizeof(industrial::shared_types::shared_int) + sizeof(industrial::shared_types::shared_int) + MAX_NUM_GROUPS * (this->joint_traj_full_sample_.byteLength() - sizeof(industrial::shared_types::shared_int));
  }

private:
  std::vector<industrial::joint_traj_pt_full::JointTrajPtFull> joint_trajectory_points_;

  industrial::joint_traj_pt_full::JointTrajPtFull joint_traj_full_sample_;
  /**
   * \brief number of groups for controllers that support multiple axis-groups
   */
  industrial::shared_types::shared_int num_groups_;
  /**
   * \brief trajectory sequence number
   */
  industrial::shared_types::shared_int sequence_;

  industrial::shared_types::shared_int valid_fields_from_message_;

  static const industrial::shared_types::shared_int MAX_NUM_GROUPS = 4;
};
}  // namespace joint_traj_pt_full_ex
}  // namespace industrial

#endif  // MOTOMAN_DRIVER_SIMPLE_MESSAGE_JOINT_TRAJ_PT_FULL_EX_H
