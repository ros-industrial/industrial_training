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

#include "input_handler.h"
#include "shared_types.h"
#include "log_wrapper.h"


//using namespace industrial::input_message;
using namespace industrial::simple_message;
using namespace industrial::shared_types;

namespace motoman
{
namespace input_handler
{


bool InputHandler::init(industrial::smpl_msg_connection::SmplMsgConnection* connection)
{
  return this->init(StandardMsgTypes::READ_INPUT, connection);
}

bool InputHandler::internalCB(industrial::simple_message::SimpleMessage & in)
{
  bool rtn = false;
  bool unloadStat = false;
  shared_int temp;
  int i = 0;
  SimpleMessage msg;
  
  do
  {
    unloadStat = in.getData().unload(temp);
    LOG_DEBUG("Message data item[%d] = %d", i, temp);
    i++;
  }
  while(unloadStat);
  
  //msg.init(StandardMsgTypes::READ_INPUT, CommTypes::SERVICE_REPLY, ReplyTypes::SUCCESS);
  //rtn = this->getConnection()->sendMsg(msg);
  rtn = true;
  
  return rtn;
}



}//namespace ping_handler
}//namespace motoman


