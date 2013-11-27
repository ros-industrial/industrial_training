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
#include "simple_message/socket/tcp_server.h"
#include "simple_message/messages/joint_message.h"

// ROS core (not really required for simple message, but allows us to use roslaunch)
#include "ros/ros.h"

// Using declarations to simplify code (BAD FORM, DON'T DO THIS)
using namespace industrial::joint_message;
using namespace industrial::tcp_server;
using namespace industrial::simple_message;

int main(int argc, char** argv)
{


  // Initialize ROS node "sm_talker"
  ros::init(argc, argv, "sm_listener");

  // Required to start timers for non-node ROS use.
  ros::Time::init();

  // Little message to know she's started
  ROS_INFO_STREAM("STARTING SM_LISTENER");

  // Create a TCP server connection on TCP_PORT (see common.h)
  TcpServer server;
  server.init(TCP_PORT);

  // While server is not connected (while loop):
  // * Print out a helpful info message
  // * Try to connect
  // * Sleep for half a second (use ROS library call: ros::Duration(0.5).sleep())
  
  // INSERT CODE HERE





  // While server is connected (while loop):
  // * Print an INFO message indicating server is waiting
  // * Receive message
  // * Check type matches JointMessage
  // * Check type for CommType::SERVICE_REQUEST
  // * Create a joint message (JointMessage) from the received simple message
  // * Print an INFO message with the sequence number
  // * Create a reply joint message (JointMessage)
  // * Convert reply to simple message (SimpleMessage)
  // * Send reply with ReplyType::SUCCESS
  // * Print an INFO message with the reply code
  
  // INSERT CODE HERE






  return 0;
}

