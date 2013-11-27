/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2011, Yaskawa America, Inc.
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
*       * Neither the name of the Yaskawa America, Inc., nor the names 
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

#ifndef __p_var_q_h
#define __p_var_q_h

#include "ros/ros.h"
#include "moto_socket.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "definitions.h"
#include "utils.h"
#include <queue>
#include <fstream>

class PVarQ
// Class for position variable queue object, used to move robot
{
  public:
    PVarQ(MotoSocket* sock, std::ofstream* log_file);
    ~PVarQ(void);
    void addTraj(const trajectory_msgs::JointTrajectory& traj);
    void sendTraj();
    void run();

  protected:
    struct message
    {
      int contents[11];
    };
    std::queue<message> messages; // Internal queue of messages to be sent to robot
    MotoSocket* sock; // Socket for communication
    std::ofstream* log_file; // Used for loggin communication
    void addMessage(int command);
    int sendMessage();
    int recvMessage();
    void logMessage(message log_message, bool is_send);
    bool ack_received;
    int num_traj_recv;
    int num_traj_send;
};

#endif
