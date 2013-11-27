/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
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

// Handler Class definition
#include "simple_message/message_handler.h"
#include "simple_message/messages/joint_message.h"

// ROS core (not really required for simple message, but allows us to use roslaunch)
#include "ros/ros.h"

// Using declarations to simplify code (BAD FORM, DON'T DO THIS)
using namespace industrial::joint_message;
using namespace industrial::simple_message;
using namespace industrial::message_handler;
using namespace industrial::smpl_msg_connection;


// Handler class defined specifically for this example
class MyHandler : public MessageHandler
{
  // since this class defines a different init(), this helps find the base-class init()
  using MessageHandler::init;

public:

  bool init(SmplMsgConnection* connection)
  {
    return this->init(StandardMsgTypes::JOINT, connection);
  }

private:

  bool internalCB(industrial::simple_message::SimpleMessage & in)
  {
    bool rtn = false;
    JointMessage jm;
    SimpleMessage msg;

    if (jm.init(in))
    {
      ROS_INFO_STREAM("Received sequence number: " << jm.getSequence());

      if (jm.toReply(msg, ReplyTypes::SUCCESS))
      {

        if (this->getConnection()->sendMsg(msg))
        {
          ROS_INFO_STREAM("Sending reply code: " << msg.getReplyCode());
          rtn = true;
        }
        else
        {
          ROS_ERROR("Failed to send joint message return");
          rtn = false;
        }
      }
      else
      {
        ROS_ERROR("Failed to generate joint reply message");
        rtn = false;
      }
    }
    else
    {
      ROS_ERROR("Failed to initialize joint message");
      rtn = false;
    }

    return rtn;
  }

};


// Common stuff for this exercise
#include "lesson_simple_message/common.h"

// Simple message
#include "simple_message/simple_message.h"
#include "simple_message/socket/tcp_server.h"
#include "simple_message/message_manager.h"


// Using declarations to simplify code (BAD FORM, DON'T DO THIS)
using namespace industrial::tcp_server;
using namespace industrial::message_manager;

int main(int argc, char** argv)
{
  // Initialize ROS node "sm_talker"
  ros::init(argc, argv, "sm_listener");

  // Required to start timers for non-node ROS use.
  ros::Time::init();

  // Little message to know she's started
  ROS_INFO_STREAM("STARTING SM_MANAGER");

  // Create and execute manager
  // * Create a TCP server connection on TCP_PORT (see common.h)
  // * Initialize manager using server connection
  // * Initialize handler using server connection
  // * Add handler to manager
  // * Execute manager spin loop
  TcpServer server;
  server.init(TCP_PORT);

  MessageManager manager;
  MyHandler handler;

  manager.init(&server);

  // Handler initilaized with reply connection (typically the same as
  // incoming connection but now always)
  handler.init(&server);
  manager.add(&handler);
  manager.spin();

  return 0;
}

