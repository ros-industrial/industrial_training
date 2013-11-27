/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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

#ifndef MOTOMAN_MOTION_REPLY_H
#define MOTOMAN_MOTION_REPLY_H

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
namespace motion_reply
{

/**
 * \brief Enumeration of motion reply result codes.
 */
namespace MotionReplyResults
{
  enum MotionReplyResult
  {
    SUCCESS    = 0,
    TRUE       = 0,
    BUSY       = 1,
    FAILURE    = 2,
    FALSE      = 2,
    INVALID    = 3,
    ALARM      = 4,
    NOT_READY  = 5,
    MP_FAILURE = 6
  };
}
typedef MotionReplyResults::MotionReplyResult MotionReplyResult;

/*
 * \brief Enumeration of Motion reply subcodes
 */
namespace MotionReplySubcodes
{
namespace Invalid
{
enum InvalidCode
{
  UNSPECIFIED = 3000,
  MSGSIZE,
  MSGHEADER,
  MSGTYPE,
  GROUPNO,
  SEQUENCE,
  COMMAND,
  DATA = 3010,
  DATA_START_POS,
  DATA_POSITION,
  DATA_SPEED,
  DATA_ACCEL,
  DATA_INSUFFICIENT
};
}

namespace NotReady
{
enum NotReadyCode
{
  UNSPECIFIED = 5000,
  ALARM,
  ERROR,
  ESTOP,
  NOT_PLAY,
  NOT_REMOTE,
  SERVO_OFF,
  HOLD,
  NOT_STARTED,
  WAITING_ROS
};
}
} // MotionReplySubcodes

/**
 * \brief Class encapsulated motion control reply data.  These messages are sent
 * by the FS100 controller in response to MotionCtrl messages.  These control
 * commands are motoman-specific.
 *
 * The byte representation of a motion control reply is as follows
 * (in order lowest index to highest). The standard sizes are given,
 * but can change based on type sizes:
 *
 *   member:             type                                      size
 *   robot_id            (industrial::shared_types::shared_int)    4  bytes
 *   sequence            (industrial::shared_types::shared_int)    4  bytes
 *   command             (industrial::shared_types::shared_int)    4  bytes
 *   result              (industrial::shared_types::shared_int)    4  bytes
 *   subcode             (industrial::shared_types::shared_int)    4  bytes
 *   data[10]            (industrial::shared_types::shared_real)   40 bytes
 *
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */

class MotionReply : public industrial::simple_serialize::SimpleSerialize
{
public:
  /**
   * \brief Default constructor
   *
   * This method creates empty data.
   *
   */
  MotionReply(void);
  /**
   * \brief Destructor
   *
   */
  ~MotionReply(void);

  /**
   * \brief Initializes a empty motion control reply
   *
   */
  void init();

  /**
   * \brief Initializes a complete motion control reply
   *
   */
  void init(industrial::shared_types::shared_int robot_id,
            industrial::shared_types::shared_int sequence,
            industrial::shared_types::shared_int command,
            MotionReplyResult result,
            industrial::shared_types::shared_int subcode,
            industrial::shared_types::shared_real data);

  /**
   * \brief Sets robot_id
   *
   * \param robot_id target robot/group # for this reply
   */
  void setRobotID(industrial::shared_types::shared_int robot_id)
  {
    this->robot_id_ = robot_id;
  }

  /**
   * \brief Returns target robot/group # for this reply
   *
   * \return robot_id
   */
  industrial::shared_types::shared_int getRobotID() const
  {
    return this->robot_id_;
  }

  /**
   * \brief Sets motion-control reply's sequence number
   *
   * \param sequence value
   */
  void setSequence(industrial::shared_types::shared_int sequence)
  {
    this->sequence_ = sequence;
  }

  /**
   * \brief Returns motion-control reply's sequence number
   *
   * \return sequence number
   */
  industrial::shared_types::shared_int getSequence() const
  {
    return this->sequence_;
  }

  /**
   * \brief Sets motion-control command (for reference)
   *
   * \param command motion-control command value
   */
  void setCommand(industrial::shared_types::shared_int command)
  {
    this->command_ = command;
  }

  /**
   * \brief Returns motion-control command
   *
   * \return motion-control command value
   */
  industrial::shared_types::shared_int getCommand() const
  {
    return this->command_;
  }

  /**
   * \brief Sets motion-control result code
   *
   * \param result motion-control result code
   */
  void setResult(industrial::shared_types::shared_int result)
  {
    this->result_ = result;
  }

  /**
   * \brief Returns motion-control result code
   *
   * \return motion-control result code
   */
  industrial::shared_types::shared_int getResult() const
  {
    return this->result_;
  }

  /**
   * \brief Sets motion-control result sub-code
   *
   * \param result motion-control result sub-code
   */
  void setSubcode(industrial::shared_types::shared_int subcode)
  {
    this->subcode_ = subcode;
  }

  /**
   * \brief Returns motion-control result sub-code
   *
   * \return motion-control result sub-code
   */
  industrial::shared_types::shared_int getSubcode() const
  {
    return this->subcode_;
  }

  /**
   * \brief Clears reply data
   */
  void clearData()
  {
    for (size_t i=0; i<MAX_DATA_CNT; ++i)
      this->data_[i] = 0.0;
  }

  /**
   * \brief Sets reply data
   *
   * \param idx  index to set
   * \param val data value
   */
  void setData(size_t idx, industrial::shared_types::shared_real val)
  {
    if (idx<MAX_DATA_CNT)
      this->data_[idx] = val;
    else
      LOG_ERROR("MotionReply data index out-of-range (%d >= %d)",
                (int)idx, (int)MAX_DATA_CNT);
  }

  /**
   * \brief Returns reply data
   *
   * \param idx data-index to get
   * \return data value
   */
  industrial::shared_types::shared_real getData(size_t idx) const
  {
    if (idx<MAX_DATA_CNT)
    {
      return this->data_[idx];
    }
    else
    {
      LOG_ERROR("MotionReply data index out-of-range (%d >= %d)",
                (int)idx, (int)MAX_DATA_CNT);

      return 0;
    }
  }

  /*
   * \brief Returns a string interpretation of a result code
   * \param code result code
   * \return string message associated with result code
   */
  static std::string getResultString(industrial::shared_types::shared_int code);
  std::string getResultString() const { return getResultString(this->result_); }

  /*
   * \brief Returns a string interpretation of a result subcode
   * \param code result subcode
   * \return string message associated with result subcode
   */
  static std::string getSubcodeString(industrial::shared_types::shared_int code);
  std::string getSubcodeString() const { return getSubcodeString(this->subcode_); }


  /**
   * \brief Copies the passed in value
   *
   * \param src (value to copy)
   */
  void copyFrom(MotionReply &src);

  /**
   * \brief == operator implementation
   *
   * \return true if equal
   */
  bool operator==(MotionReply &rhs);

  // Overrides - SimpleSerialize
  bool load(industrial::byte_array::ByteArray *buffer);
  bool unload(industrial::byte_array::ByteArray *buffer);
  unsigned int byteLength()
  {
    return 5*sizeof(industrial::shared_types::shared_int) +
           MAX_DATA_CNT*sizeof(industrial::shared_types::shared_real);
  }

private:

  /**
   * \brief Robot/group ID.
   *          0 = 1st robot
   */
  industrial::shared_types::shared_int robot_id_;

  /**
   * \brief Message-tracking number (from corresponding MotionCtrl command)
   */
  industrial::shared_types::shared_int sequence_;

  /**
   * \brief Motion-control command (from corresponding MotionCtrl command)
   */
  industrial::shared_types::shared_int command_;

  /**
   * \brief Command result
   */
  industrial::shared_types::shared_int result_;

  /**
   * \brief Command result sub-code (more detailed status)
   */
  industrial::shared_types::shared_int subcode_;

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

}
}
}

#endif /* MOTOMAN_MOTION_REPLY_H */
