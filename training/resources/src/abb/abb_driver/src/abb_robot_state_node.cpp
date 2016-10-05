/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
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

#include "abb_driver/abb_utils.h"
#include "industrial_robot_client/robot_state_interface.h"
#include "industrial_utils/param_utils.h"

using industrial_robot_client::robot_state_interface::RobotStateInterface;
using industrial_robot_client::joint_relay_handler::JointRelayHandler;

class ABB_JointRelayHandler : public JointRelayHandler
{
  bool J23_coupled_;

public:
  ABB_JointRelayHandler() : JointRelayHandler()
  {
    if (ros::param::has("J23_coupled"))
      ros::param::get("J23_coupled", this->J23_coupled_);
    else
      J23_coupled_ = false;
  }

  bool transform(const std::vector<double>& pos_in, std::vector<double>* pos_out)
  {
    // correct for parallel linkage effects, if desired
    //   - use NEGATIVE factor for motor->joint correction
    abb::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? -1:0 );

    return true;
  }
};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "state_interface");

  // launch the default Robot State Interface connection/handlers
  RobotStateInterface rsi;
  rsi.init();

  // replace the JointRelayHandler with ABB-version
  ABB_JointRelayHandler jointHandler;  // for joint-linkage correction
  std::vector<std::string> joint_names = rsi.get_joint_names();
  jointHandler.init(rsi.get_connection(), joint_names);
  rsi.add_handler(&jointHandler);

  // run the node
  rsi.run();

  return 0;
}
