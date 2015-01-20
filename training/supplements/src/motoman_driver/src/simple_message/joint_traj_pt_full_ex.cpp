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

#include <vector>

#ifndef FLATHEADERS
#include "motoman_driver/simple_message/joint_traj_pt_full_ex.h"
#include "simple_message/joint_traj_pt_full.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#else
#include "joint_traj_pt_full_ex.h"
#include "joint_traj_pt_full.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

using industrial::joint_traj_pt_full::JointTrajPtFull;
namespace ValidFieldTypes = industrial::joint_traj_pt_full::ValidFieldTypes;

namespace industrial
{
namespace joint_traj_pt_full_ex
{

JointTrajPtFullEx::JointTrajPtFullEx(void)
{
  this->init();
}
JointTrajPtFullEx::~JointTrajPtFullEx(void)
{
}

void JointTrajPtFullEx::init()
{
  this->num_groups_ = MAX_NUM_GROUPS;
  this->sequence_ = 0;

  for (int i = 0; i < MAX_NUM_GROUPS; i++)
  {
    JointTrajPtFull joint_traj_pt_full;

    joint_traj_pt_full.init();

    this->joint_trajectory_points_.push_back(joint_traj_pt_full);
  }
}

void JointTrajPtFullEx::init(industrial::shared_types::shared_int num_groups,
                             industrial::shared_types::shared_int sequence,
                             std::vector<industrial::joint_traj_pt_full::JointTrajPtFull> joint_trajectory_points)
{
  this->setNumGroups(num_groups);
  this->setSequence(sequence);
  this->setMultiJointTrajPtData(joint_trajectory_points_);
}

void JointTrajPtFullEx::copyFrom(JointTrajPtFullEx &src)
{
  this->setNumGroups(src.num_groups_);
  this->setSequence(src.sequence_);
  this->setMultiJointTrajPtData(src.joint_trajectory_points_);
}

bool JointTrajPtFullEx::operator==(JointTrajPtFullEx &rhs)
{
  // TODO(thiagodefreitas): expand the capabilities of this check
  return this->num_groups_ == rhs.num_groups_;
}

bool JointTrajPtFullEx::load(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing joint trajectory point load");

  if (!buffer->load(this->num_groups_))
  {
    LOG_ERROR("Failed to load joint traj pt. robot_id");
    return false;
  }

  if (!buffer->load(this->sequence_))
  {
    LOG_ERROR("Failed to load joint traj. pt. sequence number");
    return false;
  }

  for (int i = 0; i < joint_trajectory_points_.size(); i++)
  {
    JointTrajPtFull traj_full = joint_trajectory_points_[i];

    if (!buffer->load(traj_full.getRobotID()))
    {
      LOG_ERROR("Failed to load joint traj pt. robot_id");
      return false;
    }

    industrial::joint_data::JointData positions;
    if (traj_full.getPositions(positions))
      this->valid_fields_from_message_ |= ValidFieldTypes::POSITION;
    else
      this->valid_fields_from_message_ &= ~ValidFieldTypes::POSITION;

    industrial::joint_data::JointData velocities;
    if (traj_full.getVelocities(velocities))
      this->valid_fields_from_message_ |= ValidFieldTypes::VELOCITY;
    else
      this->valid_fields_from_message_ &= ~ValidFieldTypes::VELOCITY;

    industrial::joint_data::JointData accelerations;
    if (traj_full.getAccelerations(accelerations))
      this->valid_fields_from_message_ |= ValidFieldTypes::ACCELERATION;
    else
      this->valid_fields_from_message_ &= ~ValidFieldTypes::ACCELERATION;

    industrial::shared_types::shared_real this_time;
    if (traj_full.getTime(this_time))
      this->valid_fields_from_message_ |= ValidFieldTypes::TIME;
    else
      this->valid_fields_from_message_ &= ~ValidFieldTypes::TIME;

    if (!buffer->load(valid_fields_from_message_))
    {
      LOG_ERROR("Failed to load joint traj. pt. valid fields");
      return false;
    }

    if (!buffer->load(this_time))
    {
      LOG_ERROR("Failed to load joint traj. pt. time");
      return false;
    }

    industrial::shared_types::shared_real pos;

    for (int j = 0; j < positions.getMaxNumJoints(); j++)
    {
      pos = positions.getJoint(j);
      if (!buffer->load(pos))
      {
        LOG_ERROR("Failed to load joint traj. pt. positions");
        return false;
      }
    }

    industrial::shared_types::shared_real vel;

    for (int j = 0; j < velocities.getMaxNumJoints(); j++)
    {
      vel = velocities.getJoint(j);
      if (!buffer->load(vel))
      {
        LOG_ERROR("Failed to load joint traj. pt. positions");
        return false;
      }
    }

    industrial::shared_types::shared_real acc;

    for (int j = 0; j < accelerations.getMaxNumJoints(); j++)
    {
      acc = accelerations.getJoint(j);
      if (!buffer->load(acc))
      {
        LOG_ERROR("Failed to load joint traj. pt. positions");
        return false;
      }
    }

    LOG_COMM("Trajectory point successfully loaded");
  }
  LOG_COMM("Trajectory point successfully loaded");
  return true;
}

bool JointTrajPtFullEx::unload(industrial::byte_array::ByteArray *buffer)
{
  LOG_COMM("Executing joint traj. pt. unload");

  for (int i = 0; i < joint_trajectory_points_.size(); i++)
  {
    if (!buffer->unload(joint_trajectory_points_[i]))
    {
      LOG_ERROR("Failed to unload joint traj. pt.");
      return false;
    }
  }
  if (!buffer->unload(this->sequence_))
  {
    LOG_ERROR("Failed to unload joint traj. pt. sequence number");
    return false;
  }

  if (!buffer->unload(this->num_groups_))
  {
    LOG_ERROR("Faild to unload joint traj. pt. num_groups");
    return false;
  }
  LOG_COMM("Joint traj. pt successfully unloaded");
  return true;
}

}  // namespace joint_traj_pt_full_ex
}  // namespace industrial


