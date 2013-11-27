/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
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

#include "ros/ros.h"
#include "industrial_robot_client/robot_state_interface.h"
#include "simple_message/simple_message.h"
#include "simple_message/socket/tcp_client.h"

using industrial_robot_client::robot_state_interface::RobotStateInterface;
using namespace industrial::simple_message;
using namespace industrial::tcp_client;
using namespace industrial::simple_socket;

int main(int argc, char** argv)
{

  const unsigned int IP_ARG_IDX = 1;
  TcpClient connection;
  ros::init(argc, argv, "state_interface");

  if (argc != 1) //Only one argument, the robot IP address is accepted
  {
    ROS_INFO("Robot state connecting to IP address: %s", argv[IP_ARG_IDX]);
    connection.init(argv[IP_ARG_IDX], StandardSocketPorts::STATE);

    std::vector<std::string> joint_names;
    joint_names.push_back("joint_s");
    joint_names.push_back("joint_l");
    joint_names.push_back("joint_e");
    joint_names.push_back("joint_u");
    joint_names.push_back("joint_r");
    joint_names.push_back("joint_b");
    joint_names.push_back("joint_t");

    RobotStateInterface rsi;
    if (rsi.init(&connection, joint_names))
    {
      rsi.run();
    }

  }
  else
  {
    ROS_ERROR("Missing command line arguments, usage: robot_state <robot ip address>");
  }

  return 0;
}

