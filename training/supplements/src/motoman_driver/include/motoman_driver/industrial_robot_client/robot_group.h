/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Fraunhofer IPA
 * Author: Thiago de Freitas
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Fraunhofer IPA, nor the names
 *  of its contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
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

#ifndef MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_ROBOT_GROUP_H
#define MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_ROBOT_GROUP_H

#include <vector>
#include <string>

class RobotGroup
{
public:
  RobotGroup() {};

  std::vector<std::string> get_joint_names()
  {
    return this->joint_names_;
  }

  std::string get_name()
  {
    return this->name_;
  }

  std::string get_ns()
  {
    return this->ns_;
  }

  int get_group_id()
  {
    return this->group_id_;
  }

  void set_name(std::string name)
  {
    this->name_ = name;
  }

  void set_ns(std::string ns)
  {
    this->ns_ = ns;
  }

  void set_group_id(int gid)
  {
    this->group_id_ = gid;
  }


  void set_joint_names(std::vector<std::string> jnames)
  {
    this->joint_names_ = jnames;
  }

protected:
  std::vector<std::string> joint_names_;
  int group_id_;
  std::string name_;
  std::string ns_;
};

#endif //  MOTOMAN_DRIVER_INDUSTRIAL_ROBOT_CLIENT_ROBOT_GROUP_H
