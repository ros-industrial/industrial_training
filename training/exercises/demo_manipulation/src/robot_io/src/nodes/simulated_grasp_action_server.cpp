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


#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include "object_manipulation_msgs/GraspHandPostureExecutionGoal.h"

using namespace object_manipulation_msgs;
using namespace actionlib;

class GraspExecutionAction
{
private:
  typedef ActionServer<GraspHandPostureExecutionAction> GEAS;
  typedef GEAS::GoalHandle GoalHandle;
public:
  GraspExecutionAction(ros::NodeHandle &n) :
    node_(n),
    action_server_(node_, "grasp_execution_action",
                   boost::bind(&GraspExecutionAction::goalCB, this, _1),
                   boost::bind(&GraspExecutionAction::cancelCB, this, _1),
                   false)
  {
    ros::NodeHandle pn("~");
    action_server_.start();
    ROS_INFO_STREAM("Grasp execution action node started");
  }

  ~GraspExecutionAction()
  {
  }

private:


  void goalCB(GoalHandle gh)
  {
    std::string nodeName = ros::this_node::getName();

    ROS_INFO("%s",(nodeName + ": Received grasping goal").c_str());

	switch(gh.getGoal()->goal)
	{
		case GraspHandPostureExecutionGoal::PRE_GRASP:
			gh.setAccepted();
			//gh.getGoal()->grasp
			ROS_INFO("%s",(nodeName + ": Pre-grasp command accepted").c_str());
			gh.setSucceeded();
			break;

		case GraspHandPostureExecutionGoal::GRASP:
			gh.setAccepted();
			ROS_INFO("%s",(nodeName + ": Executing a gripper grasp").c_str());

			// wait
			ros::Duration(0.25f).sleep();
			gh.setSucceeded();
			break;

		case GraspHandPostureExecutionGoal::RELEASE:
			gh.setAccepted();
			ROS_INFO("%s",(nodeName + ": Executing a gripper release").c_str());

			// wait
			ros::Duration(0.25f).sleep();
			gh.setSucceeded();
			break;

		default:

			ROS_INFO("%s",(nodeName + ": Unidentified grasp request, rejecting goal").c_str());
			gh.setSucceeded();
			//gh.setRejected();
			break;
	}

  }

  void cancelCB(GoalHandle gh)
  {
    std::string nodeName = ros::this_node::getName();
	ROS_INFO("%s",(nodeName + ": Canceling current grasp action").c_str());
    //gh.setAccepted();
    gh.setCanceled();
    ROS_INFO("%s",(nodeName + ": Current grasp action has been canceled").c_str());
  }

  ros::NodeHandle node_;
  GEAS action_server_;

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "grasp_execution_action_node");
	ros::NodeHandle node;
	GraspExecutionAction ge(node);
	ros::spin();
  return 0;
}




