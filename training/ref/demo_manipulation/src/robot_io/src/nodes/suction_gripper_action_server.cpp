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
#include <object_manipulation_msgs/GraspHandPostureExecutionGoal.h>
#include <ur_msgs/SetIOState.h>
#include <ur_msgs/IOStates.h>
using namespace object_manipulation_msgs;
using namespace actionlib;


static const std::string INPUT_PIN_STATES_TOPIC = "/io_states";
static const std::string SET_OUTPUT_PIN_SERVICE = "/set_io_state";
static const unsigned int IO_PIN_COUNT = 8;
static const int DEFAULT_SUCTION_COMMAND_PIN = 0;
static const int DEFAULT_SUCTION_STATE_PIN = 1;

class SuctionGripperActionServer
{
private:
  typedef ActionServer<GraspHandPostureExecutionAction> GEAS;
  typedef GEAS::GoalHandle GoalHandle;

public:
  SuctionGripperActionServer(ros::NodeHandle &n) :
    node_(n),
    action_server_(node_, "grasp_execution_action",
                   boost::bind(&SuctionGripperActionServer::action_goal_callback, this, _1),
                   boost::bind(&SuctionGripperActionServer::action_cancel_callback, this, _1),
                   false),
   check_state_pin_(false),
   suction_command_pin_(DEFAULT_SUCTION_COMMAND_PIN),
   suction_state_pin_(DEFAULT_SUCTION_STATE_PIN)
  {

  }

  ~SuctionGripperActionServer()
  {
  }

  bool init()
  {
	    ros::NodeHandle nh("");
	    std::string nodeName = ros::this_node::getName();

	    // service client
	    service_client_ = nh.serviceClient<ur_msgs::SetIOState>(SET_OUTPUT_PIN_SERVICE);
	    while(!service_client_.waitForExistence(ros::Duration(5.0f)))
	    {
	    	ROS_INFO_STREAM(nodeName<<": Waiting for "<<SET_OUTPUT_PIN_SERVICE<<" to start");
	    }

	    if(load_parameters() )
	    {
	    	ROS_INFO_STREAM(nodeName<<": Loaded parameters.");
	    }
	    else
	    {
	    	ROS_ERROR_STREAM(nodeName<<": Did not find required ros parameters, exiting");
	    	return false;
	    }

	    if(!validate_pin_indices())
	    {
	    	ROS_ERROR_STREAM(nodeName<<": One or more parameter values are invalid");
	    	return false;
	    }

	    return true;
  }

  void start()
  {

	std::string nodeName = ros::this_node::getName();
	action_server_.start();
	ROS_INFO_STREAM(nodeName<<": Grasp execution action node started");
  }

  void test()
  {
	int pin;
	double state;
	std::string nodeName = ros::this_node::getName();
	ur_msgs::SetIOState::Request req;
	ur_msgs::SetIOState::Response res;

	while(ros::ok())
	{
		std::cout<<"\nEnter pin and state [0.0 or 1.0]: ";
		std::cin>>pin>>state;


		if(pin <0)
		{
			break;
		}
		else
		{
			req.state.pin= pin;
			req.state.state = state;

			if(service_client_.call(req,res))
			{
				ROS_INFO_STREAM(nodeName + ": command succeeded");
			}
			else
			{
				ROS_INFO_STREAM(nodeName + ": command aborted");
			}
		}
	}




  }

private:


  void action_goal_callback(GoalHandle gh)
  {
    std::string nodeName = ros::this_node::getName();

    ROS_INFO("%s",(nodeName + ": Received grasping goal").c_str());

    ur_msgs::SetIOState::Request req;
    ur_msgs::SetIOState::Response res;

	switch(gh.getGoal()->goal)
	{
		case GraspHandPostureExecutionGoal::PRE_GRASP:

			gh.setAccepted();
			ROS_INFO_STREAM(nodeName + ": Pre-grasp command accepted");

			req.state.pin= suction_command_pin_;
			req.state.state = 1.0f;

			if(service_client_.call(req,res))
			{
				gh.setSucceeded();
				ROS_INFO_STREAM(nodeName + ": Pre-grasp command succeeded");
			}
			else
			{
				gh.setAborted();
				ROS_INFO_STREAM(nodeName + ": Pre-grasp command aborted");
			}


			break;

		case GraspHandPostureExecutionGoal::GRASP:

			gh.setAccepted();
			ROS_INFO_STREAM(nodeName + ": Grasp command accepted");

			req.state.pin= suction_command_pin_;
			req.state.state = 1.0f;

			if(service_client_.call(req,res))
			{
				if(check_state_pin_ && !check_sensor_state())
				{
					gh.setAborted();
					ROS_INFO_STREAM(nodeName + ": Grasp check failed");
					break;
				}
			}
			else
			{
				gh.setAborted();
				ROS_INFO_STREAM(nodeName + ": Grasp command aborted");
				break;
			}

			gh.setSucceeded();
			ROS_INFO_STREAM(nodeName + ": Grasp command succeeded");
			break;

		case GraspHandPostureExecutionGoal::RELEASE:

			gh.setAccepted();
			ROS_INFO_STREAM(nodeName + ": Release command accepted");

			req.state.pin= suction_command_pin_;
			req.state.state = 0.0f;

			if(service_client_.call(req,res))
			{
				gh.setSucceeded();
				ROS_INFO_STREAM(nodeName + ": Release command succeeded");
			}
			else
			{
				gh.setAborted();
				ROS_INFO_STREAM(nodeName + ": Release command aborted");
			}

			break;

		default:

			ROS_WARN_STREAM(nodeName + ": Unidentified grasp request, rejecting request");
			gh.setRejected();
			break;
	}

  }

  void action_cancel_callback(GoalHandle gh)
  {
    std::string nodeName = ros::this_node::getName();
	ROS_INFO_STREAM(nodeName + ": Canceling current grasp action");
    gh.setCanceled();
    ROS_INFO_STREAM(nodeName + ": Current grasp action has been canceled");
  }

  bool load_parameters()
  {
	  ros::NodeHandle nh;

	  bool success = true;
	  success =  nh.getParam("use_sensor_feedback",check_state_pin_);
	  success = success && nh.getParam("suction_on_output_channel",suction_command_pin_);
	  success = success && nh.getParam("suction_check_output_channel",suction_state_pin_);
	  return success;
  }

  bool validate_pin_indices()
  {

	  if(suction_command_pin_ >= IO_PIN_COUNT)
	  {
		  return false;
	  }

	  if(suction_state_pin_ >= IO_PIN_COUNT)
	  {
		  return false;
	  }

	  return true;
  }

  bool check_sensor_state()
  {
	  ros::NodeHandle nh("");
	  ur_msgs::IOState pin_state;
	  ur_msgs::IOStatesConstPtr msg =
			  ros::topic::waitForMessage<ur_msgs::IOStates>(INPUT_PIN_STATES_TOPIC,nh,ros::Duration(2.0f));
	  if(!msg)
	  {
		  ROS_ERROR_STREAM(ros::this_node::getName()<<": IOStates message not received");
		  return true;
	  }
	  else
	  {
		  pin_state = msg->states[suction_state_pin_];
		  return (( pin_state.state >  0.0f) ? true : false);
	  }

  }

  // ros comm
  ros::NodeHandle node_;
  GEAS action_server_;
  ros::ServiceClient service_client_;

  // ros parameters
  int suction_command_pin_; // index value to output channel for suction
  int suction_state_pin_; // index value to input channel for vacuum sensor
  bool check_state_pin_;

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "grasp_execution_action_node");
	ros::AsyncSpinner spinner(4);
	spinner.start();
	ros::NodeHandle nh("");
	SuctionGripperActionServer grasp_action_server(nh);
	if(grasp_action_server.init())
	{
		//grasp_action_server.test();
		grasp_action_server.start();
	}

	ros::waitForShutdown();
  return 0;
}




