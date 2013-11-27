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


#include "ros/ros.h"
#include "simple_message/socket/tcp_client.h"
#include "simple_message/simple_message.h"
#include "simple_message/ping_message.h"
#include <iostream>

using namespace industrial::simple_socket;
using namespace industrial::tcp_client;
using namespace industrial::simple_message;
using namespace industrial::ping_message;


int main(int argc, char** argv)
// Allows user to send commands to the robot
{
  char ip[1024] = "192.168.10.3"; // Robot IP address
  bool exit = false;
  SimpleMessage reply, request;
  PingMessage tReply, tRequest;
  TcpClient connection;

  ROS_INFO("Setting up tcp client");
  connection.init(ip, StandardSocketPorts::SYSTEM);

  ROS_INFO("Connecting to server");
  if (connection.makeConnect())
  {
    tRequest.init();
    tRequest.toRequest(request);
    
    do {
    ROS_INFO("Sending ping");
    connection.sendAndReceiveMsg(request, reply);
    ROS_INFO("Ping recieved");
    std::cout << "Type 1 to exit" << std::endl;
    std::cin >> exit;
	} while (!exit);

    tReply.init(reply);
  }



  /*
  int command;
  int reply_message[11];

  ros::init(argc, argv, "system_interface");
  ros::NodeHandle n;

  MotoSocket sock(buff, SYSTEM_PORT);

  while (ros::ok())
  {
    cout << "Enter command number (0 to exit) ==> ";
    cin >> command;
    if (command == 0)
      break;
    sock.sendMessage(command, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, UNUSED, true);
    sock.recvMessage(reply_message, true);
    for (short i = 0; i < 11; i++)
      cout << reply_message[i] << " ";
    cout << endl;
  }
  */
  return 0;
}
