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

#ifndef MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_MOTION_CTRL_H
#define MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_MOTION_CTRL_H

#ifdef ROS
#include "simple_message/simple_serialize.h"
#include "simple_message/shared_types.h"
#include "simple_message/log_wrapper.h"
#endif

#ifdef MOTOPLUS
#include "simple_serialize.h"
#include "shared_types.h"
#include "log_wrapper.h"
#endif

namespace motoman
{
namespace simple_message
{
namespace motion_ctrl
{
/**
 * \brief Enumeration of motion control command codes.
 */
namespace MotionControlCmds
{
enum MotionControlCmd
{
  UNDEFINED          = 0,
  CHECK_MOTION_READY = 200101,  // check if controller is ready to receive ROS motion cmds
  CHECK_QUEUE_CNT    = 200102,  // get number of motion increments in queue
  STOP_MOTION        = 200111,  // stop robot motion immediately
  START_TRAJ_MODE    = 200121,  // prepare controller to receive ROS motion cmds
  STOP_TRAJ_MODE     = 200122,  // return motion control to INFORM
};
}  // namespace MotionControlCmds
typedef MotionControlCmds::MotionControlCmd MotionControlCmd;

/**
 * \brief Class encapsulated motion control data.  These control messages are
 * required to download command-trajectories to the FS100 controller.  Trajectory
 * data is provided in a separate JointTrajPtFull message.  These control
 * commands are motoman-specific.
 *
 * The byte representation of a motion control command is as follows
 * (in order lowest index to highest). The standard sizes are given,
 * but can change based on type sizes:
 *
 *   member:             type                                      size
 *   robot_id            (industrial::shared_types::shared_int)    4  bytes
 *   sequence            (industrial::shared_types::shared_int)    4  bytes
 *   command             (industrial::shared_types::shared_int)    4  bytes
 *   data[10]            (industrial::shared_types::shared_real)   40 bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class MotionCtrl : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  MotionCtrl(void);
  /**
   * \brief Destructor
   *
   */
  ~MotionCtrl(void);

  /**
   * \brief Initializes a empty motion control command
   *
   */
  void init();

  /**
   * \brief Initializes a complete motion control command
   *
   */
  void init(industrial::shared_types::shared_int robot_id,
            industrial::shared_types::shared_int sequence,
            MotionControlCmd command,
            industrial::shared_types::shared_real data);

  /**
   * \brief Sets robot_id
   *
   * \param robot_id target robot/group # for this command
   */
  void setRobotID(industrial::shared_types::shared_int robot_id)
  {
    this->robot_id_ = robot_id;
  }

  /**
   * \brief Returns target robot/group # for this command
   *
   * \return robot_id
   */
  industrial::shared_types::shared_int getRobotID()
  {
    return this->robot_id_;
  }

  /**
   * \brief Sets control command's sequence number
   *
   * \param sequence value
   */
  void setSequence(industrial::shared_types::shared_int sequence)
  {
    this->sequence_ = sequence;
  }

  /**
   * \brief Returns control command's sequence number
   *
   * \return control command sequence number
   */
  industrial::shared_types::shared_int getSequence()
  {
    return this->sequence_;
  }

  /**
   * \brief Sets motion control command
   *
   * \param command motion-control command value
   */
  void setCommand(MotionControlCmd command)
  {
    this->command_ = command;
  }

  /**
   * \brief Returns motion control command
   *
   * \return motion-control command value
   */
  MotionControlCmd getCommand()
  {
    return (MotionControlCmd)this->command_;
  }

  /**
   * \brief Clears command data
   */
  void clearData()
  {
    for (size_t i = 0; i < MAX_DATA_CNT; ++i)
      this->data_[i] = 0.0;
  }

  /**
   * \brief Sets command data
   *
   * \param idx  index to set
   * \param val data value
   */
  void setData(size_t idx, industrial::shared_types::shared_real val)
  {
    if (idx < MAX_DATA_CNT)
      this->data_[idx] = val;
    else
      LOG_ERROR("MotionCtrl data index out-of-range (%d >= %d)",
                static_cast<int>(idx), static_cast<int>(MAX_DATA_CNT));
  }

  /**
   * \brief Returns command data
   *
   * \param idx data-index to get
   * \return data value
   */
  industrial::shared_types::shared_real getData(size_t idx)
  {
    if (idx < MAX_DATA_CNT)
    {
      return this->data_[idx];
    }
    else
    {
      LOG_ERROR("MotionCtrl data index out-of-range (%d >= %d)",
                static_cast<int>(idx), static_cast<int>(MAX_DATA_CNT));

      return 0;
    }
  }

  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(MotionCtrl &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(MotionCtrl &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 3 * sizeof(industrial::shared_types::shared_int) +
           MAX_DATA_CNT * sizeof(industrial::shared_types::shared_real);
  }

private:
  /**
   * \brief Robot/group ID.
   *          0 = 1st robot
   */
  industrial::shared_types::shared_int robot_id_;

  /**
   * \brief Message-tracking number that will be echoed back in reply message
   */
  industrial::shared_types::shared_int sequence_;

  /**
   * \brief Motion-control command
   */
  industrial::shared_types::shared_int command_;

  /**
   * \brief Maximum length (# of float elements) of data buffer
   */
  static const size_t MAX_DATA_CNT = 10;

  /**
   * \brief Motion-control command data
   *        Contents of data-buffer are specific to each command.
   */
  industrial::shared_types::shared_real data_[MAX_DATA_CNT];
};
}  // namespace motion_ctrl
}  // namespace simple_message
}  // namespace motoman

#endif /* MOTOMAN_DRIVER_SIMPLE_MESSAGE_MOTOMAN_MOTION_CTRL_H */
