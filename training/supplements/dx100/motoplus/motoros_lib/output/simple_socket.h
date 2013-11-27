/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
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
 *       * Neither the name of the Southwest Research Institute, nor the names
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

#ifndef SIMPLE_SOCKET_H
#define SIMPLE_SOCKET_H

#ifdef ROS

#include "sys/socket.h"
#include "arpa/inet.h"
#include "string.h"
#include "unistd.h"
#include "netinet/tcp.h"
#include "errno.h"

#include "simple_message/log_wrapper.h"
#include "simple_message/shared_types.h"
#include "simple_message/smpl_msg_connection.h"

#define SOCKET(domain, type, protocol) socket(domain, type, protocol)
#define BIND(sockfd, addr, addrlen) bind(sockfd, addr, addrlen)
#define SET_NO_DELAY(sockfd, val) setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &val, sizeof(val))
#define SET_REUSE_ADDR(sockfd, val) setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val))
#define LISTEN(sockfd, n) listen(sockfd, n)
#define ACCEPT(sockfd, addr, addrlen) accept(sockfd, addr, addrlen)
#define CONNECT(sockfd, dest_addr ,addrlen) connect(sockfd, dest_addr, addrlen)
#define SEND_TO(sockfd, buf, len, flags, dest_addr, addrlen) sendto(sockfd, buf, len, flags, dest_addr, addrlen)
#define SEND(sockfd, buf, len, flags) send(sockfd, buf, len, flags)
#define RECV_FROM(sockfd, buf, len, flags, src_addr, addrlen) recvfrom(sockfd, buf, len, flags, src_addr, addrlen)
#define RECV(sockfd, buf, len, flags) recv(sockfd, buf, len, flags)
#define SELECT(n, readfds, writefds, exceptfds, timeval) select(n, readfds, writefds, exceptfds, timeval)
#define CLOSE(fd) close(fd)
#ifndef HTONS // OSX defines HTONS
#define HTONS(num) htons(num)
#endif
#define INET_ADDR(str) inet_addr(str)
#define SOCKLEN_T socklen_t

#endif

#ifdef MOTOPLUS

#include "motoPlus.h"

#include "errno.h"
#include "log_wrapper.h"
#include "shared_types.h"
#include "smpl_msg_connection.h"

// Including os defintion for set socket option.  The motoplus wrappers do not give access to socket
// options.  In order to remove system delays the nagel algorithm must be disabled using the
// TPC_NO_DELAY option
extern "C" STATUS setsockopt (   /* remove "extern C", if you're using C instead of C++ */
    int    s,                 /* target socket */
    int    level,             /* protocol level of option */
    int    optname,           /* option name */
    char * optval,            /* pointer to option value */
    int    optlen             /* option length */
    );

#define SOCKET(domain, type, protocol) mpSocket(domain, type, protocol)
#define BIND(sockfd, addr, addrlen) mpBind(sockfd, addr, addrlen)

// Motoplus compliant version (i.e. a NOOP)
// #define SET_NO_DELAY(sockfd, val) -1 //MOTOPLUS does not allow for setting the "no delay" socket option
// Raw OS call, not Motoplus compliant and might not be allowed in future versions. (taking a risk at this point)
#define SET_NO_DELAY(sockfd, val) setsockopt(sockfd, SOL_SOCKET, TCP_NODELAY, (char *)&val, sizeof(val))

#define SET_REUSE_ADDR(sockfd, val) -1 //MOTOPLUS does not support this function.
#define LISTEN(sockfd, n) mpListen(sockfd, n)
#define ACCEPT(sockfd, addr, addrlen) mpAccept(sockfd, addr, addrlen)
#define CONNECT(sockfd, dest_addr ,addrlen) mpConnect(sockfd, dest_addr, addrlen)
#define SEND_TO(sockfd, buf, len, flags, dest_addr, addrlen) mpSendTo(sockfd, buf, len, flags, dest_addr, addrlen)
#define SEND(sockfd, buf, len, flags) mpSend(sockfd, buf, len, flags)
#define RECV_FROM(sockfd, buf, len, flags, src_addr, addrlen) mpRecvFrom(sockfd, buf, len, flags, src_addr, (int*)addrlen)
#define RECV(sockfd, buf, len, flags) mpRecv(sockfd, buf, len, flags)
#define SELECT(n, readfds, writefds, exceptfds, timeval) mpSelect(n, readfds, writefds, exceptfds, timeval)
#define CLOSE(fd) mpClose(fd)
#define HTONS(num) mpHtons(num)
#define INET_ADDR(str) mpInetAddr(str)
#define SOCKLEN_T unsigned int

#endif

namespace industrial
{
namespace simple_socket
{

/**
 * \brief Enumeration of standard socket ports (supported by all platforms).
 * These are defined for convenience.  Other ports may be used.  Additional
 * ports for application specific needs may also be defined.
 */
namespace StandardSocketPorts
{
enum StandardSocketPort
{
  MOTION = 11000, SYSTEM = 11001, STATE = 11002, IO = 11003
};
}
typedef StandardSocketPorts::StandardSocketPort StandardSocketPort;

/**
 * \brief Defines socket functions required for a simple connection type.
 */
class SimpleSocket : public industrial::smpl_msg_connection::SmplMsgConnection
{
public:

  /**
     * \brief Constructor
     */
  SimpleSocket(){}

  /**
     * \brief Destructor
     */
  virtual ~SimpleSocket(){}

  bool isConnected()
  {
    return connected_;
  }

protected:

  /**
   * \brief socket handle for sending/receiving data
   */
  int sock_handle_;

  /**
   * \brief address/port of remote socket
   */
  sockaddr_in sockaddr_;
  
  /**
   * \brief flag indicating socket connection status
   */
  bool connected_;

  /**
   * \brief socket fail return value
   */
  static const int SOCKET_FAIL = -1;

  /**
   * \brief maximum size of buffer for receiving data (fixed memory size used
   * in order to avoid dynamic memory allocation)
   */
  static const int MAX_BUFFER_SIZE = 1024;
  /**
   * \brief internal data buffer for receiving
   */
  char buffer_[MAX_BUFFER_SIZE + 1];

  int  getSockHandle() const
  {
    return sock_handle_;
  }

  void setSockHandle(int sock_handle_)
  {
    this->sock_handle_ = sock_handle_;
  }
  
  virtual void setConnected(bool connected)
  {
    this->connected_ = connected;
  }

  void logSocketError(char* msg, int rc)
  {
    LOG_ERROR("%s, rc: %d, errno: %d", msg, rc, errno);
  }
  
  /**
   * \brief returns true if socket data is ready to receive
   *
   * \param timeout (ms) negative or zero values result in blocking
   *
   * \return true if data is ready to recieve
   */
  bool isReadyReceive(int timeout);
  
  // Send/Receive functions (inherited classes should override raw methods
  // Virtual
  bool sendBytes(industrial::byte_array::ByteArray & buffer);
  bool receiveBytes(industrial::byte_array::ByteArray & buffer,
      industrial::shared_types::shared_int num_bytes);
  // Virtual
  virtual int rawSendBytes(char *buffer,
      industrial::shared_types::shared_int num_bytes)=0;
  virtual int rawReceiveBytes(char *buffer,
      industrial::shared_types::shared_int num_bytes)=0;

};

} //simple_socket
} //industrial

#endif /* SIMPLE_SOCKET_H */
