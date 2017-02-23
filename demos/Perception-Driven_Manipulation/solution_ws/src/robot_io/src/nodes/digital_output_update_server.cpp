/*
 * ouput_state_service_server.cpp
 *
 *  Created on: Dec 14, 2012
 *      Author: jnicho
 */

#include <ros/ros.h>
#include <robot_io/DigitalOutputUpdate.h>
#include <soem_beckhoff_drivers/DigitalMsg.h>

static const std::string OUTPUT_TOPIC = "/digital_outputs";
static const std::string OUTPUT_SERVICE = "/digital_output_update";
static const std::string SERVER_RUNNING_PARAM = "/digital_output_server_running";

class DigitalOutputUpdateServer
{
public:

	DigitalOutputUpdateServer()
	{

	}

	~DigitalOutputUpdateServer()
	{
		// set param to false
		ros::NodeHandle nh("/");
		bool serverRunning = false;
		if(nh.getParam(SERVER_RUNNING_PARAM,serverRunning) && serverRunning)
		{
			nh.setParam(SERVER_RUNNING_PARAM,false);
		}
	}

	bool serviceCallback(robot_io::DigitalOutputUpdate::Request &req,robot_io::DigitalOutputUpdate::Response &res)
	{
		if(validateRequest(req))
		{
			output_msg_.values[req.bit_index] = (bool)req.output_bit_state;
			pub_.publish(output_msg_);

			ros::Duration(0.5f).sleep();
			res.output_bit_array = output_msg_.values;
			return true;
		}
		else
		{
			ROS_ERROR_STREAM(ros::this_node::getName()<<": Rejected output update request");
			return false;
		}
	}

	void init()
	{
		ros::NodeHandle nh("/");
	    std::string nodeName = ros::this_node::getName();

	    ROS_INFO_STREAM(nodeName<<": Grasp execution action node started");
		pub_ = nh.advertise<soem_beckhoff_drivers::DigitalMsg>(OUTPUT_TOPIC, 1);

		while ( pub_.getNumSubscribers() <= 0 && ros::ok())
		{
			ros::Duration(5.0).sleep();
		    ROS_INFO_STREAM(nodeName<<": Waiting for digital output subscribers");
		}

		// initializing and sending output message
		output_msg_.values.resize(robot_io::DigitalOutputUpdate::Request::COUNT, false);
		output_msg_.values[robot_io::DigitalOutputUpdate::Request::COLLISION] = true;
		pub_.publish(output_msg_);
	    ROS_INFO_STREAM(nodeName<<": Turning on air pressure to collision sensor");

		server_ = nh.advertiseService(OUTPUT_SERVICE,&DigitalOutputUpdateServer::serviceCallback,this);
		ROS_INFO_STREAM(nodeName<<": Advertising "<<OUTPUT_SERVICE<<" service");

		nh.setParam(SERVER_RUNNING_PARAM,true);
	}

	static bool checkSingletonInstanceRunning()
	{
		ros::NodeHandle nh("/");
		bool server_running = false;
		nh.getParam(SERVER_RUNNING_PARAM,server_running);
		return server_running;
	}

protected:

	  // ros comm
	  ros::NodeHandle node_;
	  ros::ServiceServer server_;
	  ros::Publisher pub_;

	  // messages
	  soem_beckhoff_drivers::DigitalMsg output_msg_;

	  bool validateRequest(const robot_io::DigitalOutputUpdate::Request &req)
	  {
		  if(req.bit_index >= req.COUNT|| req.bit_index == req.COLLISION)
		  {
			  return false;
		  }

		  return true;
	  }

};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"digital_output_update_server");
	ros::NodeHandle nh("/"); // will use global namespaces

	if(DigitalOutputUpdateServer::checkSingletonInstanceRunning())
	{
		ROS_WARN_STREAM(ros::this_node::getName()<<": Server is already running, only one instance is allowed");
		return 0;
	}

	DigitalOutputUpdateServer server;
	server.init();
	ros::spin();

	return 0;
}
