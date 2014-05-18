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

#ifndef FLATHEADERS
#include "simple_message/socket/udp_socket.h"
#include "simple_message/log_wrapper.h"
#include "simple_message/simple_message.h"
#else
#include "udp_socket.h"
#include "log_wrapper.h"
#include "simple_message.h"
#endif


using namespace industrial::smpl_msg_connection;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::shared_types;

namespace industrial
{
namespace udp_socket
{

UdpSocket::UdpSocket()
// Constructor for UDP socket object
{
  this->setSockHandle(this->SOCKET_FAIL);
  memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));

}

UdpSocket::~UdpSocket()
// Destructor for UDP socket object
// Closes socket
{
  CLOSE(this->getSockHandle());
}

bool UdpSocket::receiveMsg(SimpleMessage & message)
{
  ByteArray msgBuffer;

  bool rtn = false;
  shared_int size = 0;

  rtn = this->receiveBytes(msgBuffer, 0);

  if (rtn)
  {
    LOG_DEBUG("Receive message bytes: %u", msgBuffer.getBufferSize());
    if (msgBuffer.getBufferSize() >= sizeof(shared_int))
    {
	    LOG_DEBUG("Unloading message length from front of the buffer");
	    msgBuffer.unloadFront((void*)(&size), sizeof(shared_int));

	    if ( size != (shared_int) msgBuffer.getBufferSize() )
	    {
	      LOG_WARN("readBytes returned a message other than the expected size");
	    }
	    rtn = message.init(msgBuffer);

	    if (rtn)
	    {
	      rtn = true;
	    }
	    else
	    {
	      LOG_ERROR("Failed to initialize message");
	      rtn = false;
	    }
     }
     else
     {
        LOG_ERROR("Receive bytes returned small: %d message", rtn);
        LOG_ERROR("Possible handshake or other connection issue, setting disconnected");
        this->setConnected(false);
        rtn = false;
     }

  }
  else
  {
    LOG_ERROR("Failed to receive message");
    rtn = false;
  }

  return rtn;
}

int UdpSocket::rawSendBytes(char *buffer, shared_int num_bytes)
{
  int rc = this->SOCKET_FAIL;

  rc = SEND_TO(this->getSockHandle(), buffer,
        num_bytes, 0, (sockaddr *)&this->sockaddr_,
        sizeof(this->sockaddr_));
  
  return rc;
}

int UdpSocket::rawReceiveBytes(char *buffer, shared_int num_bytes)
{
  int rc = this->SOCKET_FAIL;
  SOCKLEN_T addrSize = 0;

  addrSize = sizeof(this->sockaddr_);

  rc = RECV_FROM(this->getSockHandle(), &this->buffer_[0], this->MAX_BUFFER_SIZE,
      0, (sockaddr *)&this->sockaddr_, &addrSize);
  
  return rc;
}


} //udp_socket
} //industrial

