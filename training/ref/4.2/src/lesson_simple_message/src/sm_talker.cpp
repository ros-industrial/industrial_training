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

// Common stuff for this exercise
#include "lesson_simple_message/common.h"

// Simple message
#include "simple_message/simple_message.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/messages/joint_message.h"

// ROS core (not really required for simple message, but allows us to use roslaunch)
#include "ros/ros.h"

// Using declarations to simplify code (BAD FORM, DON'T DO THIS)
using namespace industrial::joint_message;
using namespace industrial::tcp_client;
using namespace industrial::simple_message;

int main(int argc, char** argv)
{
  // Initialize ROS node "sm_talker"
  ros::init(argc, argv, "sm_talker");

  // Required to start timers for non-node ROS use.
  ros::Time::init();

  // Little message to know she's started
  ROS_INFO_STREAM("STARTING SM_TALKER");

  // Create a TCP client connection to TCP_PORT on LOCAL_HOST (see common.h)
  TcpClient client;
  client.init(LOCAL_HOST, TCP_PORT);

  // While client is not connected (while loop):
  // * Print out a helpful info message
  // * Try to connect
  // * Sleep for half a second (use ROS library call: ros::Duration(0.5).sleep())
  while(!client.isConnected())
  {
    ROS_INFO_STREAM("Trying to connect to server");
    client.makeConnect();
    ros::Duration(0.5).sleep();
  }

  int seq = 0;

  // While client is connected (while loop):
  // * increment sequence number of joint message
  // * Create a joint message (JointMessage)
  // * Create a simple message (SimpleMessage) request from joint message
  // * Print an INFO message with sequence number
  // * send message to server and wait for reply
  // * Print an INFO message with the reply code
  // * Sleep for 1.0 seconds: ros::Duration(1.0).sleep();

  while(client.isConnected())
  {
    // Create a message of type JointMessage and corresponding simple message
    JointMessage jmReq;
    SimpleMessage req, reply;


    seq++;
    jmReq.setSequence(seq);

    jmReq.toRequest(req);

    ROS_INFO_STREAM("Sending sequence number: " << seq);

    client.sendAndReceiveMsg(req, reply);

    ROS_INFO_STREAM("Received reply code: " << reply.getReplyCode());

    ros::Duration(1.0).sleep();

  }

  return 0;
}
