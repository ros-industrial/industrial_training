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
#include "simple_message/socket/simple_socket.h"
#include "simple_message/log_wrapper.h"
#else
#include "simple_socket.h"
#include "log_wrapper.h"
#endif

using namespace industrial::byte_array;
using namespace industrial::shared_types;

namespace industrial
{
namespace simple_socket
{


bool SimpleSocket::sendBytes(ByteArray & buffer)
{
  int rc = this->SOCKET_FAIL;
  bool rtn = false;

  if (this->isConnected())
  {
    // Nothing restricts the ByteArray from being larger than the what the socket
    // can handle.
    if (this->MAX_BUFFER_SIZE > (int)buffer.getBufferSize())
    {
    
      rc = rawSendBytes(buffer.getRawDataPtr(), buffer.getBufferSize());
      if (this->SOCKET_FAIL != rc)
      {
        rtn = true;
      }
      else
      {
        rtn = false;
        logSocketError("Socket sendBytes failed", rc);
      }
      
      }
    else
    {
      LOG_ERROR("Buffer size: %u, is greater than max socket size: %u", buffer.getBufferSize(), this->MAX_BUFFER_SIZE);
      rtn = false;
    }

  }
  else
  {
    rtn = false;
    LOG_WARN("Not connected, bytes not sent");
  }

  if (!rtn)
    {
      this->setConnected(false);
    }

  return rtn;
  
}

bool SimpleSocket::receiveBytes(ByteArray & buffer, shared_int num_bytes)
{
  int rc = this->SOCKET_FAIL;
  bool rtn = false;
  
  // Reset the buffer (this is not required since the buffer length should
  // ensure that we don't read any of the garbage that may be left over from
  // a previous read), but it is good practice.

  memset(&this->buffer_, 0, sizeof(this->buffer_));

  // Doing a sanity check to determine if the byte array buffer is larger than
  // what can be sent in the socket.  This should not happen and might be indicative
  // of some code synchronization issues between the client and server base.
  if (this->MAX_BUFFER_SIZE < (int)buffer.getMaxBufferSize())
  {
    LOG_WARN("Socket buffer max size: %u, is larger than byte array buffer: %u",
             this->MAX_BUFFER_SIZE, buffer.getMaxBufferSize());
  }
  if (this->isConnected())
  {
    rc = rawReceiveBytes(this->buffer_, num_bytes);

    if (this->SOCKET_FAIL != rc)
    {
      if (rc > 0)
      {
        LOG_COMM("Byte array receive, bytes read: %u", rc);
        buffer.init(&this->buffer_[0], rc);
        rtn = true;
      }
      else
      {
        LOG_WARN("Recieved zero bytes: %u", rc);
        rtn = false;
      }
    }
    else
    {
      this->logSocketError("Socket received failed", rc);
      rtn = false;
    }
  }
  else
  {
    rtn = false;
    LOG_WARN("Not connected, bytes not sent");
  }

  if (!rtn)
  {
    this->setConnected(false);
  }
  return rtn;
}

bool SimpleSocket::isReadyReceive(int timeout)
{
  timeval time;
  fd_set read, write, except;
  int rc = this->SOCKET_FAIL;
  bool rtn = false;
  
  // The select function uses the timeval data structure
  time.tv_sec = timeout/1000;
  time.tv_usec = (timeout%1000)*1000;
  
  FD_ZERO(&read);
  FD_ZERO(&write);
  FD_ZERO(&except);
  
  FD_SET(this->getSockHandle(), &read);
  
  rc = SELECT(this->getSockHandle() + 1, &read, &write, &except, &time);
  
  if (this->SOCKET_FAIL != rc)
  {
    if (0==rc)
    {
      LOG_DEBUG("Socket select timed out");
      rtn = false;
    }
    else
    {
      LOG_DEBUG("Data is ready for reading");
      rtn = true;
    }
  }
  else
  {
    this->logSocketError("Socket select function failed", rc);
    rtn = false;
  }
  
  return rtn;
}



} //simple_socket
} //industrial

