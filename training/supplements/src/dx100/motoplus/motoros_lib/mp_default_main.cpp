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

#include "controller.h"
#include "mp_default_main.h"
#include "log_wrapper.h"
#include "tcp_server.h"
#include "message_manager.h"
#include "input_handler.h"
#include "joint_motion_handler.h"
#include "joint_data.h"
#include "joint_message.h"
#include "robot_status.h"
#include "robot_status_message.h"
#include "simple_message.h"
#include "ros_conversion.h"

#include "motoPlus.h"


namespace motoman
{
namespace mp_default_main
{

void motionServer(void)
// Persistent TCP server that receives motion messages from Motoros node (ROS interface) and relays to parseMotionMessage
{

    using namespace industrial::simple_socket;
    using namespace industrial::tcp_server;
    using namespace industrial::message_manager;
    using namespace industrial::simple_message;
    using namespace motoman::joint_motion_handler;
  
    TcpServer connection;
    JointMotionHandler jmHandler;
    
    MessageManager manager;
    
    connection.init(StandardSocketPorts::MOTION);
    

    connection.makeConnect();
    
    manager.init(&connection);
    
    jmHandler.init(StandardMsgTypes::JOINT, &connection);
    manager.add(&jmHandler);
    manager.spin();


}


void systemServer(void)
// Persistent TCP server that receives system messages from Motoros node (ROS interface) and relays to parseSystemMessage
{

    using namespace industrial::simple_socket;
    using namespace industrial::tcp_server;
    using namespace industrial::message_manager;
    
    TcpServer connection;
    MessageManager manager;
    
    connection.init(StandardSocketPorts::SYSTEM);
    connection.makeConnect();
    
    manager.init(&connection);
    manager.spin();

}



void stateServer(void)
{
    using motoman::controller::Controller;
    using namespace industrial::simple_socket;
    using namespace industrial::tcp_server;
    using namespace industrial::joint_message;
    using namespace industrial::joint_data;
    using namespace industrial::robot_status;
    using namespace industrial::robot_status_message;
    using namespace industrial::simple_message;
    using namespace motoman::ros_conversion;
    
    // Using TPC server for debugging (this should really be UDP)
    TcpServer connection;
    
    JointData rosJoints;
    JointMessage msg;
    
    RobotStatus status;
    RobotStatusMessage statusMsg;
    
    SimpleMessage simpMsg;
    Controller controller;
    float mpJoints[MAX_PULSE_AXES];
    
    const int period = 100; //ticks
    
    connection.init(StandardSocketPorts::STATE);
    
    FOREVER
    {
      connection.makeConnect();
      
      while(connection.isConnected())
      {
        controller.getActJointPos(mpJoints);
        toRosJoint(mpJoints, rosJoints);
        msg.init(0, rosJoints);
        msg.toTopic(simpMsg);
        connection.sendMsg(simpMsg);
        
        controller.getStatus(status);
        statusMsg.init(status);
        statusMsg.toTopic(simpMsg);
        connection.sendMsg(simpMsg);
        
        mpTaskDelay(period);
        
        
      }
      

    }

}



void ioServer(void)
{
/*

    using namespace industrial::simple_socket;
    using namespace industrial::tcp_server;
    using namespace industrial::message_manager;
    using namespace industrial::simple_message;
    using namespace motoman::input_handler;
    
    TcpServer connection;
    InputHandler iHandler;
    MessageManager manager;
    
    connection.init(StandardSocketPorts::IO);
    connection.makeConnect();
    
    manager.init(&connection);
    
    iHandler.init(StandardMsgTypes::WRITE_OUTPUT, &connection);
    manager.add(&iHandler);
    manager.spin();
 */

    
}


} //mp_wrapper
} //motoman
