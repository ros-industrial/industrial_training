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

#include "industrial_robot_client/robot_state_interface.h"
#include "industrial_utils/param_utils.h"

using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial_utils::param::getJointNames;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

namespace industrial_robot_client
{
namespace robot_state_interface
{

RobotStateInterface::RobotStateInterface()
{
  this->connection_ = NULL;
  this->add_handler(&default_joint_handler_);
  this->add_handler(&default_robot_status_handler_);
}

bool RobotStateInterface::init()
{
  ros::NodeHandle n;
  std::string s;

  // initialize default connection, if one not specified.
  if (!n.getParam("robot_ip_address", s))
  {
    ROS_ERROR("Robot State failed to get param 'robot_ip_address'");
    return false;
  }

  char* ip_addr = strdup(s.c_str());  // connection.init() requires "char*", not "const char*"
  ROS_INFO("Robot state connecting to IP address: %s", ip_addr);
  default_tcp_connection_.init(ip_addr, StandardSocketPorts::STATE);
  free(ip_addr);

  return init(&default_tcp_connection_);
}

bool RobotStateInterface::init(SmplMsgConnection* connection)
{
  std::vector<std::string> joint_names;
  if (!getJointNames("controller_joint_names", joint_names))
    ROS_WARN("Unable to read 'controller_joint_names' param.  Using standard 6-DOF joint names.");

  return init(connection, joint_names);
}

bool RobotStateInterface::init(SmplMsgConnection* connection, std::vector<std::string>& joint_names)
{
  this->connection_ = connection;
  connection_->makeConnect();

  // initialize message-manager
  if (!manager_.init(connection_))
    return false;

  // initialize default handlers
  if (!default_joint_handler_.init(connection_, joint_names))
    return false;
  this->add_handler(&default_joint_handler_);

  if (!default_robot_status_handler_.init(connection_))
      return false;
  this->add_handler(&default_robot_status_handler_);

  return true;
}

void RobotStateInterface::run()
{
  manager_.spin();
}

} // robot_state_interface
} // industrial_robot_client
